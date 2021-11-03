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
ElevationMapProcessor::ElevationMapProcessor(ros::NodeHandle &p_nh):
    m_nh(p_nh)
{
    // Start thread
    m_workerThread = std::thread([this] {processElevationMap();} );

    // Elevation map subscriber
    m_elevationMapSubscriber = m_nh.subscribe(HEIGHT_MAP_TOPIC,
                                              10,
                                              &ElevationMapProcessor::elevationMapCallback,
                                              this);
}

/**
 * Destructor.
 */
ElevationMapProcessor::~ElevationMapProcessor()
{}

/**
 * Elevation map callback.
 *
 * @param p_elevationMap
 */
void ElevationMapProcessor::elevationMapCallback(const grid_map_msgs::GridMap &p_elevationMap)
{
    // Clean queue
    m_mutex.lock();
    if (m_elevationMapQueue.size() > 3)
    {
        m_elevationMapQueue.pop();
    }

    m_elevationMapQueue.push(p_elevationMap);
    m_mutex.unlock();

    // Process the newest elevation map
    processElevationMap();
}

/**
 * Post process elevation map
 * in order to compute cost map.
 *
 * @param p_elevationMap
 */
void ElevationMapProcessor::processElevationMap()
{
    m_mutex.lock();
    // Get latest elevation map
    const grid_map_msgs::GridMap l_elevationMapMsg = m_elevationMapQueue.front();
    m_mutex.unlock();

    // Convert grid_map_msgs to grid_map
    grid_map::GridMap l_elevationMap;
    if (!grid_map::GridMapRosConverter::fromMessage(l_elevationMapMsg, l_elevationMap))
    {
        ROS_ERROR("ElevationMapProcessor: Could not convert grid_map_msgs to grid_map.");
    }

    // Convert elevation map to OpenCV
    cv::Mat l_elevationMapImage;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(l_elevationMap,
                                                             "elevation",
                                                             CV_8UC1,
                                                             l_elevationMapImage);

    // Change possible NaN values in the map to 0
    cv::patchNaNs(l_elevationMapImage, 0);

    // Apply sobel filter to elevation map image
    cv::Mat l_sobelX, l_sobelY;
    cv::Sobel(l_elevationMapImage, l_sobelX, CV_8U, 1, 0, 3);
    cv::Sobel(l_elevationMapImage, l_sobelY, CV_8U, 0, 1, 3);

    // Element wise maximum
    cv::Mat l_sobelMax = cv::max(l_sobelX, l_sobelY);

    // Set costmap traversability
    cv::Mat l_costmap;
    cv::threshold(l_sobelMax, l_costmap, 0.5, 1, cv::THRESH_BINARY);

    //TODO: populate queue with processed elevation map
}


