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

using namespace std::chrono;

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

    /**
     * Obtain the latest processed foot costmap
     * and the latest acquired elevation map.
     */
    std::tuple<cv::Mat, grid_map::Matrix> getCostmaps();
private:
    /**
     * Elevation map callback that processes
     * the incoming elevation map and computes
     * the costmap.
     *
     * @param p_elevationMapMsg
     */
    void elevationMapCallback(const grid_map_msgs::GridMap &p_elevationMapMsg);

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! ROS publishers
    ros::Publisher m_elevationMapProcessedPublisher;

    //! ROS subscribers
    ros::Subscriber m_elevationMapSubscriber;

    //! Elevation map caches
    std::queue<cv::Mat> m_footCostmaps;
    std::queue<grid_map::Matrix> m_elevationMaps;

    //! Mutex for processed elevation maps queue
    std::mutex m_mutex;
};