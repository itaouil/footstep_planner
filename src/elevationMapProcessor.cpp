/*
 * elevationMapProcessor.cpp
 *
 *  Created on: Nov 2, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "elevationMapProcessor.hpp"

/**
 * Constructor.
 *
 * @param p_nh
 */
ElevationMapProcessor::ElevationMapProcessor(ros::NodeHandle &p_nh) :
        m_nh(p_nh),
        m_setElevationMapParameters(false) {
    // Publishers
    m_elevationMapPublisher = m_nh.advertise<grid_map_msgs::GridMap>("elevation_processed", 1);
    m_costmapPublisher = m_nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);

    // Elevation map subscriber
    m_elevationMapSubscriber = m_nh.subscribe(HEIGHT_MAP_TOPIC,
                                              10,
                                              &ElevationMapProcessor::elevationMapCallback,
                                              this);
}

/**
 * Destructor.
 */
ElevationMapProcessor::~ElevationMapProcessor() = default;

/**
 * Elevation map callback.
 *
 * @param p_elevationMap
 */
void ElevationMapProcessor::elevationMapCallback(const grid_map_msgs::GridMap &p_elevationMapMsg) {
    // Only set the resolution and the
    // map grid sizes once as these are
    // static and don't change at runtime
    if (!m_setElevationMapParameters) {
        m_elevationMapGridResolution = p_elevationMapMsg.info.resolution;
        m_elevationMapGridSizeX = static_cast<unsigned int>(
                p_elevationMapMsg.info.length_x / m_elevationMapGridResolution);
        m_elevationMapGridSizeY = static_cast<unsigned int>(
                p_elevationMapMsg.info.length_y / m_elevationMapGridResolution);

        m_setElevationMapParameters = true;
    }

    // Always reset the grid map origin
    // as it keeps changing while the
    // robot moves
    m_elevationMapGridOriginX = p_elevationMapMsg.info.pose.position.x;
    m_elevationMapGridOriginY = p_elevationMapMsg.info.pose.position.y;

    // Convert grid_map_msgs to grid_map
    grid_map::GridMap l_elevationMap;
    if (!grid_map::GridMapRosConverter::fromMessage(p_elevationMapMsg, l_elevationMap)) {
        ROS_ERROR("ElevationMapProcessor: Could not convert grid_map_msgs to grid_map.");
    }

    // Convert elevation map to OpenCV
    cv::Mat l_elevationMapImage;
    if (!grid_map::GridMapCvConverter::toImage<unsigned char, 1>(l_elevationMap,
                                                                 "elevation_inpainted",
                                                                 CV_8UC1,
                                                                 l_elevationMap.get(
                                                                         "elevation_inpainted").minCoeffOfFinites(),
                                                                 l_elevationMap.get(
                                                                         "elevation_inpainted").maxCoeffOfFinites(),
                                                                 l_elevationMapImage)) {
        ROS_ERROR("ElevationMapProcessor: Could not convert grid_map to cv::Mat.");
    }

    // Blur elevation map image to reduce noise
    cv::Mat l_elevationMapBlurred;
    cv::GaussianBlur(l_elevationMapImage, l_elevationMapBlurred, cv::Size(3, 3), 0, 0);

//    cv::imshow("Original Image", l_elevationMapImage);
//    cv::imshow("Blurred Image", l_elevationMapBlurred);
//    cv::waitKey(0);

    // Change possible NaN values in the map to 0
//    cv::patchNaNs(l_elevationMapImage, 0.0);

//    cv::imshow("Elevation Map OpenCV image (nans patched)", l_elevationMapImage);
//    cv::waitKey(0);

    // Dilate elevation map image to fill sparse regions
//    cv::Mat l_elevationMapImageDilated;
//    cv::dilate(l_elevationMapImage, l_elevationMapImageDilated, cv::Mat());
//
//    cv::imshow("Elevation Map OpenCV image (dilated)", l_elevationMapImageDilated);
//    cv::waitKey(0);

    // Erode elevation map to eliminate extraneous sensor returns
//    cv::Mat l_elevationMapImageEroded;
//    cv::dilate(l_elevationMapImageDilated, l_elevationMapImageEroded, cv::Mat());
//
//    cv::imshow("Elevation Map OpenCV image (eroded)", l_elevationMapImageEroded);
//    cv::waitKey(0);
//

    // Apply sobel filter to elevation map image
    cv::Mat l_sobelX, l_sobelY;
    cv::Sobel(l_elevationMapImage, l_sobelX, CV_32F, 1, 0, 3);
    cv::Sobel(l_elevationMapImage, l_sobelY, CV_32F, 0, 1, 3);

    // Build overall filter result by combining the previous results
    cv::Mat l_sobelMagnitude;
    cv::magnitude(l_sobelX, l_sobelY, l_sobelMagnitude);

    // Set costmap traversability based on computed gradients
    cv::Mat l_costmap;
    cv::threshold(l_sobelMagnitude, l_costmap, GRADIENT_THRESHOLD, 1, cv::THRESH_BINARY);

    // Flip costmap around the horizontal
    // axis and subsequently rotate it to
    // obtain it in the same orientation as
    // the original elevation map
    cv::Mat l_costmapFlipped;
    cv::Mat l_costmapRotated;
    cv::flip(l_costmap, l_costmapFlipped, 1);
    cv::rotate(l_costmapFlipped, l_costmapRotated, cv::ROTATE_90_CLOCKWISE);

    // Add new costmaps and the relative
    // elevation map eigen matrix
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);

        if (m_footCostmaps.size() > 4) {
            m_footCostmaps.pop();
            m_elevationMaps.pop();
        }

        m_footCostmaps.push(l_costmap);
        m_elevationMaps.push(l_elevationMap.get("elevation_inpainted"));
    }

    if (PUBLISH) {
        std::vector<float> l_data;
        if (l_costmapRotated.isContinuous()) {
            l_data.assign((float *) l_costmapRotated.data,
                          (float *) l_costmapRotated.data + l_costmapRotated.total() * l_costmapRotated.channels());
        } else {
            for (int i = 0; i < l_sobelMagnitude.rows; ++i) {
                l_data.insert(l_data.end(), l_costmapRotated.ptr<float>(i),
                              l_costmapRotated.ptr<float>(i) + l_costmapRotated.cols * l_costmapRotated.channels());
            }
        }

        // Create occupancy grid from computed gradients
        nav_msgs::OccupancyGrid l_occupancyGrid;
        l_occupancyGrid.header = p_elevationMapMsg.info.header;
        l_occupancyGrid.info.resolution = static_cast<float>(p_elevationMapMsg.info.resolution);
        l_occupancyGrid.info.width = static_cast<int>(p_elevationMapMsg.info.length_x /
                                                      l_occupancyGrid.info.resolution);
        l_occupancyGrid.info.height = static_cast<int>(p_elevationMapMsg.info.length_y /
                                                       l_occupancyGrid.info.resolution);
        l_occupancyGrid.info.origin.position.x =
                p_elevationMapMsg.info.pose.position.x - p_elevationMapMsg.info.length_x / 2;
        l_occupancyGrid.info.origin.position.y =
                p_elevationMapMsg.info.pose.position.y - p_elevationMapMsg.info.length_y / 2;
        l_occupancyGrid.info.origin.position.z = 0.0;

        // Populate occupancy grid (for visualization purposes only)
        for (auto &i: l_data) {
            if (static_cast<int>(i) > 0)
                l_occupancyGrid.data.push_back(100);
            else
                l_occupancyGrid.data.push_back(0);
        }

        // Create color layer to visualize
        // costmap on the elevation layer
        cv::Mat3b l_colorLayerBGR(l_costmap.rows, l_costmap.cols, CV_8UC3);
        for (int i = 0; i < l_costmap.rows; i++) {
            for (int j = 0; j < l_costmap.cols; j++) {
                // Steppable cell
                if (static_cast<int>(l_costmap.at<float>(i, j)) == 1) {
                    l_colorLayerBGR(i, j)[0] = 172;
                    l_colorLayerBGR(i, j)[1] = 0;
                    l_colorLayerBGR(i, j)[2] = 0;
                }
                    // Unsteppable cell
                else {
                    l_colorLayerBGR(i, j)[0] = 0;
                    l_colorLayerBGR(i, j)[1] = 172;
                    l_colorLayerBGR(i, j)[2] = 0;
                }
            }
        }

        // Add color layer
        grid_map::GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(l_colorLayerBGR, "color",
                                                                               l_elevationMap);

        // Publish occupancy grid
        m_costmapPublisher.publish(l_occupancyGrid);

        // Publish elevation map with colored layer
        grid_map_msgs::GridMap l_newElevationMapMsg;
        grid_map::GridMapRosConverter::toMessage(l_elevationMap, l_newElevationMapMsg);
        m_elevationMapPublisher.publish(l_newElevationMapMsg);
    }
}

/**
 * Check if predicted feet configuration
 * is valid (i.e. stepping on valid terrain).
 *
 * @param p_row
 * @param p_col
 * @return true if valid, otherwise false
 */
bool ElevationMapProcessor::checkFootholdValidity(const int &p_row, const int &p_col) {
    // Obtain latest costmap
    cv::Mat l_latestCostmap;
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        l_latestCostmap = m_footCostmaps.back();
    }

    return static_cast<bool>(l_latestCostmap.at<float>(p_row, p_col));
}

/**
 * Obtain latest processed costmap.
 */
std::tuple<cv::Mat, grid_map::Matrix> ElevationMapProcessor::getCostmaps() {
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        return std::make_tuple(m_footCostmaps.back(), m_elevationMaps.back());
    }
}

/**
 * Obtain the elevation map parameters,
 * including the grid resolution and its
 * sizes.
 *
 * @param p_elevationMapGridOriginX
 * @param p_elevationMapGridOriginY
 * @param p_elevationMapGridResolution
 * @param p_elevationMapGridSizeX
 * @param p_elevationMapGridSizeY
 */
void ElevationMapProcessor::getElevationMapParameters(double &p_elevationMapGridOriginX,
                                                      double &p_elevationMapGridOriginY,
                                                      double &p_elevationMapGridResolution,
                                                      unsigned int &p_elevationMapGridSizeX,
                                                      unsigned int &p_elevationMapGridSizeY) {
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        p_elevationMapGridOriginX = m_elevationMapGridOriginX;
        p_elevationMapGridOriginY = m_elevationMapGridOriginY;
        p_elevationMapGridResolution = m_elevationMapGridResolution;
        p_elevationMapGridSizeX = m_elevationMapGridSizeX;
        p_elevationMapGridSizeY = m_elevationMapGridSizeY;
    }
}

/**
 * Obtain the latest grid map origin
 * as it updates upon every robot
 * displacement.
 *
 * @param p_elevationMapGridOriginX
 * @param p_elevationMapGridOriginY
 */
void ElevationMapProcessor::getUpdatedElevationMapGridOrigin(double &p_elevationMapGridOriginX,
                                                             double &p_elevationMapGridOriginY) {
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        p_elevationMapGridOriginX = m_elevationMapGridOriginX;
        p_elevationMapGridOriginY = m_elevationMapGridOriginY;
    }
}