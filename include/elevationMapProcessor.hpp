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
#include <tf2_ros/transform_listener.h>

// Structs
#include <structs/feetConfiguration.hpp>

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
     * Returns the height of a
     * given cell index.
     *
     * @param p_row
     * @param p_col
     * @return true if access was successful, otherwise false
     */
    double getCellHeight(const int &p_row, const int &p_col);

    /**
     * Check if predicted feet configuration
     * is valid (i.e. stepping on valid terrain).
     *
     * @param p_row
     * @param p_col
     * @return true if valid, otherwise false
     */
    bool checkFootholdValidity(const int &p_row, const int &p_col);

    /**
     * Obtain the latest processed foot costmap
     * and the latest acquired elevation map.
     */
    std::tuple<cv::Mat, grid_map::Matrix> getCostmaps();

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
    void getElevationMapParameters(double &p_elevationMapGridOriginX,
                                   double &p_elevationMapGridOriginY,
                                   double &p_elevationMapGridResolution,
                                   unsigned int &p_elevationMapGridSizeX,
                                   unsigned int &p_elevationMapGridSizeY);

    /**
     * Obtain the latest grid map origin
     * as it updates upon every robot
     * displacement.
     *
     * @param p_elevationMapGridOriginX
     * @param p_elevationMapGridOriginY
     */
    void getUpdatedElevationMapGridOrigin(double &p_elevationMapGridOriginX, double &p_elevationMapGridOriginY);
private:
    /**
     * Elevation map callback that processes
     * the incoming elevation map and computes
     * the costmap.
     *
     * @param p_elevationMapMsg
     */
    void elevationMapCallback(const grid_map_msgs::GridMap &p_elevationMapMsg);

    //! Mutex for processed elevation maps queue
    std::mutex m_mutex;

    //! ROS node handle
    ros::NodeHandle m_nh;

    //! Elevation map parameters
    bool m_setElevationMapParameters;
    double m_elevationMapGridOriginX;
    double m_elevationMapGridOriginY;
    double m_elevationMapGridResolution;
    unsigned int m_elevationMapGridSizeX;
    unsigned int m_elevationMapGridSizeY;

    //! Height and foot costs caches
    std::queue<cv::Mat> m_footCostmaps;
    std::queue<grid_map::Matrix> m_elevationMaps;

    //! ROS subscribers
    ros::Subscriber m_elevationMapSubscriber;

    //! ROS publishers
    ros::Publisher m_costmapPublisher;
    ros::Publisher m_elevationMapPublisher;
};