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

    // Elevation grid map threaded postprocessing
    m_thread = std::thread(&ElevationMapProcessor::gridMapPostProcessing, this);
}

/**
 * Destructor.
 */
ElevationMapProcessor::~ElevationMapProcessor() {
    // Stop thread
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

/**
 * Elevation map callback.
 *
 * @param p_gridMapMsg
 */
void ElevationMapProcessor::elevationMapCallback(const grid_map_msgs::GridMap &p_gridMapMsg) {
    // Only set the resolution and the
    // map grid sizes once as these are
    // static and don't change at runtime
    if (!m_setElevationMapParameters) {
        m_elevationMapGridResolution = p_gridMapMsg.info.resolution;
        m_elevationMapGridSizeX = static_cast<unsigned int>(
                p_gridMapMsg.info.length_x / m_elevationMapGridResolution);
        m_elevationMapGridSizeY = static_cast<unsigned int>(
                p_gridMapMsg.info.length_y / m_elevationMapGridResolution);

        m_setElevationMapParameters = true;
    }

    // Always reset the grid map origin
    // as it keeps changing while the
    // robot moves
    m_elevationMapGridOriginX = p_gridMapMsg.info.pose.position.x;
    m_elevationMapGridOriginY = p_gridMapMsg.info.pose.position.y;

    // Add latest elevation map to queue
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);

        if (m_gridMapMsgs.size() > 4) {
            m_gridMapMsgs.pop();
        }

        m_gridMapMsgs.push(p_gridMapMsg);
    }
}

/**
 * Routine that processes the raw elevation
 * maps in order to compute the traversability
 * costmap and the post processed elevation map.
 */
void ElevationMapProcessor::gridMapPostProcessing() {
    while (ros::ok()) {
        // Skip if no elevation grid map acquired yet
        if (m_gridMapMsgs.empty()) {
            continue;
        }

        // Acquire latest elevation grid map
        grid_map_msgs::GridMap l_elevationGridMapMsg;
        {
            std::lock_guard<std::mutex> l_lockGuard(m_mutex);
            l_elevationGridMapMsg = m_gridMapMsgs.back();
        }

        // Convert grid_map_msgs to grid_map
        grid_map::GridMap l_elevationMap;
        if (!grid_map::GridMapRosConverter::fromMessage(l_elevationGridMapMsg, l_elevationMap)) {
            ROS_ERROR("ElevationMapProcessor: Could not convert grid_map_msgs to grid_map.");
        }

        // Convert elevation map to OpenCV
        cv::Mat l_elevationMapImage;
        if (!grid_map::GridMapCvConverter::toImage<float, 1>(l_elevationMap,
                                                             "elevation",
                                                             CV_32F,
                                                             l_elevationMap.get(
                                                                     "elevation").minCoeffOfFinites(),
                                                             l_elevationMap.get(
                                                                     "elevation").maxCoeffOfFinites(),
                                                             l_elevationMapImage)) {
            ROS_ERROR("ElevationMapProcessor: Could not convert grid_map to cv::Mat.");
        }

        // Dilate elevation map image to fill sparse regions
        cv::Mat l_elevationMapImageDilated;
        cv::dilate(l_elevationMapImage, l_elevationMapImageDilated, cv::Mat());

        // Erode elevation map to eliminate extraneous sensor returns
        cv::Mat l_elevationMapImageEroded;
        cv::erode(l_elevationMapImageDilated, l_elevationMapImageEroded, cv::Mat());

        // Apply sobel filter to elevation map image
        cv::Mat l_sobelX, l_sobelY;
        cv::Sobel(l_elevationMapImageEroded, l_sobelX, CV_32F, 1, 0, 3);
        cv::Sobel(l_elevationMapImageEroded, l_sobelY, CV_32F, 0, 1, 3);

        // Build overall filter result by combining the previous results
        cv::Mat l_sobelMagnitude;
        cv::magnitude(l_sobelX, l_sobelY, l_sobelMagnitude);

        // Set costmap traversability based on computed gradients
        // such that traversable cells are set to 1 and impassable
        // ones to 0
        cv::Mat l_costmap;
        cv::threshold(l_sobelMagnitude, l_costmap, GRADIENT_THRESHOLD, 1, cv::THRESH_BINARY_INV);

        // Compute distance transform
        cv::Mat l_distanceTransform;
        cv::Mat l_converted8UC1Costmap;
        l_costmap.convertTo(l_converted8UC1Costmap, CV_8UC1);
        cv::distanceTransform(l_converted8UC1Costmap, l_distanceTransform, cv::DIST_L2, 3);

        // Add traversability, elevation and distance
        // data to the respective queues in a thread safe
        // manner
        {
            std::lock_guard<std::mutex> l_lockGuard(m_mutex);

            if (m_traversabilityCostmaps.size() > 4) {
                m_distanceTransforms.pop();
                m_traversabilityCostmaps.pop();
            }

            m_traversabilityCostmaps.push(l_costmap);
            m_distanceTransforms.push(l_distanceTransform);
            m_gridMaps.push(l_elevationMap.get("elevation_inpainted"));
        }

        // Visualize occupancy grid and colored
        // elevation map based on traversability
        if (PUBLISH) {
            // Flip costmap around the horizontal
            // axis and subsequently rotate it to
            // obtain it in the same orientation as
            // the original elevation map
            cv::Mat l_costmapFlipped;
            cv::Mat l_costmapRotated;
            cv::flip(l_costmap, l_costmapFlipped, 1);
            cv::rotate(l_costmapFlipped, l_costmapRotated, cv::ROTATE_90_CLOCKWISE);

            // Create vector from traversability costmap
            std::vector<float> l_traversabilityData;
            if (l_costmapRotated.isContinuous()) {
                l_traversabilityData.assign((float *) l_costmapRotated.data,
                                            (float *) l_costmapRotated.data +
                                            l_costmapRotated.total() * l_costmapRotated.channels());
            } else {
                for (int i = 0; i < l_sobelMagnitude.rows; ++i) {
                    l_traversabilityData.insert(l_traversabilityData.end(), l_costmapRotated.ptr<float>(i),
                                                l_costmapRotated.ptr<float>(i) +
                                                l_costmapRotated.cols * l_costmapRotated.channels());
                }
            }

            // Create occupancy grid from computed gradients
            nav_msgs::OccupancyGrid l_occupancyGrid;
            l_occupancyGrid.header = l_elevationGridMapMsg.info.header;
            l_occupancyGrid.info.resolution = static_cast<float>(l_elevationGridMapMsg.info.resolution);
            l_occupancyGrid.info.width = static_cast<int>(l_elevationGridMapMsg.info.length_x /
                                                          l_occupancyGrid.info.resolution);
            l_occupancyGrid.info.height = static_cast<int>(l_elevationGridMapMsg.info.length_y /
                                                           l_occupancyGrid.info.resolution);
            l_occupancyGrid.info.origin.position.x =
                    l_elevationGridMapMsg.info.pose.position.x - l_elevationGridMapMsg.info.length_x / 2;
            l_occupancyGrid.info.origin.position.y =
                    l_elevationGridMapMsg.info.pose.position.y - l_elevationGridMapMsg.info.length_y / 2;
            l_occupancyGrid.info.origin.position.z = 0.0;

            // Populate occupancy grid (for visualization purposes only)
            for (auto &cellTraversability: l_traversabilityData) {
                // Empty cell
                if (static_cast<int>(cellTraversability))
                    l_occupancyGrid.data.push_back(0);
                    // Non-empty cell
                else
                    l_occupancyGrid.data.push_back(100);
            }

            // Create color layer to visualize
            // costmap on the elevation layer
            cv::Mat3b l_colorLayerBGR(l_costmap.rows, l_costmap.cols, CV_8UC3);
            for (int i = 0; i < l_costmap.rows; i++) {
                for (int j = 0; j < l_costmap.cols; j++) {
                    // Traversable cell
                    if (l_distanceTransform.at<float>(i, j) * m_elevationMapGridResolution >= MIN_STAIR_DISTANCE) {
                        l_colorLayerBGR(i, j)[0] = 0;
                        l_colorLayerBGR(i, j)[1] = 255;
                        l_colorLayerBGR(i, j)[2] = 0;
                    }
                        // Impassable cell
                    else {
                        l_colorLayerBGR(i, j)[0] = 255;
                        l_colorLayerBGR(i, j)[1] = 0;
                        l_colorLayerBGR(i, j)[2] = 0;
                    }
                }
            }

            // Change elevation layer to processed image
            grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(l_elevationMapImageEroded,
                                                                      "processed_elevation",
                                                                      l_elevationMap,
                                                                      l_elevationMap.get(
                                                                              "elevation").minCoeffOfFinites(),
                                                                      l_elevationMap.get(
                                                                              "elevation").maxCoeffOfFinites());

            // Add color layer
            grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(l_colorLayerBGR,
                                                                              "color",
                                                                              l_elevationMap,
                                                                              0,
                                                                              255);

            // Publish occupancy grid
            m_costmapPublisher.publish(l_occupancyGrid);

            // Publish elevation map with colored layer
            grid_map_msgs::GridMap l_newElevationMapMsg;
            grid_map::GridMapRosConverter::toMessage(l_elevationMap, l_newElevationMapMsg);
            m_elevationMapPublisher.publish(l_newElevationMapMsg);
        }
    }
}

/**
 * Returns the height of a
 * given cell index.
 *
 * @param p_row
 * @param p_col
 * @return true if access was successful, otherwise false
 */
double ElevationMapProcessor::getCellHeight(const int &p_row, const int &p_col) {
    // Obtain latest costmap
    grid_map::Matrix l_latestElevationMap;
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        l_latestElevationMap = m_gridMaps.back();
    }

    return l_latestElevationMap.coeff(p_row, p_col);
}

/**
 * Check if predicted feet configuration
 * is valid (i.e. stepping on valid terrain).
 *
 * @param p_row
 * @param p_col
 * @return true if valid, otherwise false
 */
bool ElevationMapProcessor::validFootstep(const int &p_row, const int &p_col) {
    // Obtain latest costmap
    cv::Mat l_latestDistanceTransform;
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        l_latestDistanceTransform = m_distanceTransforms.back();
    }

    return ((l_latestDistanceTransform.at<float>(p_row, p_col) * m_elevationMapGridResolution) >= MIN_STAIR_DISTANCE);
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