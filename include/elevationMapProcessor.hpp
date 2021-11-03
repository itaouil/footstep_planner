/*
 * elevationMapProcessor.hpp
 *
 *  Created on: Nov 2, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ general
#include <queue>
#include <thread>
#include <mutex>

// OpenCV
#include "opencv2/opencv.hpp"

// ROS general
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// Config file
#include "config.hpp"

class ElevationMapProcessor
{
public:
    /**
     * Constructor.
     */
    explicit ElevationMapProcessor(ros::NodeHandle& p_nh);

    /**
     * Destructor.
     */
    virtual ~ElevationMapProcessor();
private:
    /**
     * Elevation map callback.
     *
     * @param p_elevationMap
     */
    void elevationMapCallback(const grid_map_msgs::GridMap &p_elevationMap);

    /**
     * Post process elevation map
     * in order to compute cost map.
     */
    void processElevationMap();

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS subscribers
    ros::Subscriber m_elevationMapSubscriber;

    //! Elevation map caches
    std::queue<grid_map_msgs::GridMap> m_elevationMapQueue;
    std::queue<grid_map_msgs::GridMap> m_processedElevationMapQueue;

    //! Elevation map thread processor
    std::thread m_workerThread;

    //! Mutex for processed elevation maps queue
    std::mutex m_mutex;
};