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
    m_gridMapPublisher = m_nh.advertise<grid_map_msgs::GridMap>("elevation_processed", 1);

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

        grid_map_msgs::GridMap l_elevationGridMapMsg;
        {
            std::lock_guard<std::mutex> l_lockGuard(m_mutex);
            l_elevationGridMapMsg = m_gridMapMsgs.back();
        }

        grid_map::GridMap l_elevationMap;
        if (!grid_map::GridMapRosConverter::fromMessage(l_elevationGridMapMsg, l_elevationMap)) {
            ROS_ERROR("ElevationMapProcessor: Could not convert grid_map_msgs to grid_map.");
        }

        cv::Mat l_elevationMapImage;
        if (!grid_map::GridMapCvConverter::toImage<float, 1>(l_elevationMap,
                                                             "elevation",
                                                             CV_32F,
                                                             l_elevationMap["elevation"].minCoeffOfFinites(),
                                                             l_elevationMap["elevation"].maxCoeffOfFinites(),
                                                             l_elevationMapImage)) {
            ROS_ERROR("ElevationMapProcessor: Could not convert grid_map to cv::Mat.");
        }

        cv::Mat l_elevationMapImageDilated;
        cv::dilate(l_elevationMapImage, l_elevationMapImageDilated, cv::Mat());

        cv::Mat l_elevationMapImageEroded;
        cv::erode(l_elevationMapImageDilated, l_elevationMapImageEroded, cv::Mat());

        cv::Mat l_elevationMapImageFiltered;
        cv::medianBlur(l_elevationMapImageEroded, l_elevationMapImageFiltered, 3);

        cv::Mat l_sobelX, l_sobelY;
        cv::Sobel(l_elevationMapImageFiltered, l_sobelX, CV_32F, 1, 0, 3);
        cv::Sobel(l_elevationMapImageFiltered, l_sobelY, CV_32F, 0, 1, 3);

        cv::Mat l_sobelMagnitude;
        cv::magnitude(l_sobelX, l_sobelY, l_sobelMagnitude);

        // Set free cells to 1, and occupied to 0
        cv::Mat l_costmap;
        cv::threshold(l_sobelMagnitude, l_costmap, GRADIENT_THRESHOLD, 1, cv::THRESH_BINARY_INV);
        l_costmap.convertTo(l_costmap, CV_8UC1);

        cv::Mat l_costmapCV8UC1;
        cv::Mat l_distanceTransform;
        cv::distanceTransform(l_costmap, l_distanceTransform, cv::DIST_L2, 3);

        grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(l_costmap,
                                                                          "costmap",
                                                                          l_elevationMap,
                                                                          0,
                                                                          1);

        grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(l_elevationMapImage,
                                                                  "processed_elevation",
                                                                  l_elevationMap,
                                                                  l_elevationMap["elevation"].minCoeffOfFinites(),
                                                                  l_elevationMap["elevation"].maxCoeffOfFinites());

        grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(l_distanceTransform,
                                                                  "distance",
                                                                  l_elevationMap);

        {
            std::lock_guard<std::mutex> l_lockGuard(m_mutex);
            m_gridMap = l_elevationMap;
        }

        // Create color layer to visualize costmap on the elevation layer
        cv::Mat3b l_colorLayerBGR(l_costmap.rows, l_costmap.cols, CV_8UC3);
        for (int i = 0; i < l_costmap.rows; i++) {
            for (int j = 0; j < l_costmap.cols; j++) {
                // Traversable cell
                if (l_distanceTransform.at<float>(i, j) * m_elevationMapGridResolution > MIN_STAIR_DISTANCE) {
                    l_colorLayerBGR(i, j)[0] = 0;
                    l_colorLayerBGR(i, j)[1] = 0;
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

        // Add color layer
        grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(l_colorLayerBGR,
                                                                          "color",
                                                                          l_elevationMap,
                                                                          0,
                                                                          255);

        // Publish elevation map with colored layer
        grid_map_msgs::GridMap l_newElevationMapMsg;
        grid_map::GridMapRosConverter::toMessage(l_elevationMap, l_newElevationMapMsg);
        m_gridMapPublisher.publish(l_newElevationMapMsg);

        // Get callback data
        ros::spinOnce();
    }
}

/**
 * Gets the corresponding cell index for world position.
 *
 * @param p_worldCoordinates
 * @return true if successful, false if position outside of map
 */
bool ElevationMapProcessor::worldToGrid(const World3D &p_worldCoordinates, Vec2D &p_gridCoordinates) {
    bool l_successful;
    grid_map::Index l_gridCoordinates;

    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);

        // Get 2D grid index from 2D world pose
        grid_map::Position l_worldCoordinates{p_worldCoordinates.x, p_worldCoordinates.y};
        l_successful = m_gridMap.getIndex(l_worldCoordinates, l_gridCoordinates);
    }

    // Copy over cell index
    if (l_successful) {
        p_gridCoordinates.x = l_gridCoordinates.x();
        p_gridCoordinates.y = l_gridCoordinates.y();
    }

    return l_successful;
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
    // Obtain latest elevation map
    double l_cellHeight;
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        l_cellHeight = m_gridMap["processed_elevation"].coeff(p_row, p_col);
    }

    return l_cellHeight;
}

/**
 * Check if predicted feet configuration
 * is valid (i.e. stepping on valid terrain).
 *
 * @param p_nextRow
 * @param p_nextCol
 * @param p_footDistance
 * @return true if valid, otherwise false
 */
bool ElevationMapProcessor::validFootstep(const int &p_prevRow,
                                          const int &p_prevCol,
                                          const int &p_nextRow,
                                          const int &p_nextCol,
                                          float &p_footDistance) {
    bool l_safeFootstep;
    bool l_invalidFootstep;
    float l_newFootstepHeight;
    float l_prevFootstepHeight;

    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        l_safeFootstep = m_gridMap["costmap"].coeff(p_nextRow, p_nextCol);
        l_invalidFootstep = std::isnan(m_gridMap["elevation"].coeff(p_nextRow, p_nextCol));
        l_prevFootstepHeight = m_gridMap["processed_elevation"].coeff(p_prevRow, p_prevCol);
        l_newFootstepHeight = m_gridMap["processed_elevation"].coeff(p_nextRow, p_nextCol);
        p_footDistance = std::min({m_gridMap["distance"].coeff(p_nextRow, p_nextCol),
                                      m_gridMap["distance"].coeff(p_nextRow, p_nextCol)}) *
                                      m_elevationMapGridResolution;

        ROS_INFO_STREAM(m_gridMap["elevation"].coeff(p_nextRow, p_nextCol) << ", " << m_gridMap["processed_elevation"].coeff(p_nextRow, p_nextCol));
    }

    ROS_INFO_STREAM("Safe footstep: " << l_safeFootstep);
    ROS_INFO_STREAM("Invalid footstep: " << l_invalidFootstep);
    ROS_INFO_STREAM("Footstep height difference: " << abs(l_newFootstepHeight - l_prevFootstepHeight));

    return l_safeFootstep && !l_invalidFootstep && abs(l_newFootstepHeight - l_prevFootstepHeight) < MAX_FOOTSTEP_HEIGHT;
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