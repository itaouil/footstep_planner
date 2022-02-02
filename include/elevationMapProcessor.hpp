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
#include <mutex>
#include <thread>

// OpenCV
#include "opencv2/opencv.hpp"

// ROS general
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/transform_listener.h>

// Structs
#include <structs/vec2D.hpp>
#include <structs/feetConfiguration.hpp>

// Config file
#include "config.hpp"

using namespace std::chrono;

class ElevationMapProcessor {
public:
    /**
     * Constructor.
     */
    explicit ElevationMapProcessor(ros::NodeHandle &p_nh);

    /**
     * Destructor.
     */
    virtual ~ElevationMapProcessor();

    /**
     * Gets the corresponding cell index for world position.
     *
     * @param p_worldCoordinates
     * @param p_gridCoordinates
     * @return true if successful, false if position outside of map
     */
    bool worldToGrid(const World3D &p_worldCoordinates, Vec2D &p_gridCoordinates);

    /**
     * Returns the height of a
     * given cell index.
     *
     * @param p_row
     * @param p_col
     * @return the cell's height
     */
    double getCellHeight(const int &p_row, const int &p_col);

    /**
     * Check if predicted feet configuration
     * is valid (i.e. stepping on valid terrain).
     *
     * @param p_nextRow
     * @param p_nextCol
     * @return true if valid, otherwise false
     */
    bool validFootstep(const int &p_nextRow, const int &p_nextCol);

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
     * Elevation map callback.
     *
     * @param p_gridMapMsg
     */
    void elevationMapCallback(const grid_map_msgs::GridMap &p_gridMapMsg);

    /**
     * Threaded routine that processes the
     * raw elevation maps in order to compute
     * the traversability costmap and the post
     * processed elevation map.
     */
    void gridMapPostProcessing();

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

    //! Grid map messages cache
    std::queue<grid_map_msgs::GridMap> m_gridMapMsgs;

    //! Latest layer data and distance transform
    grid_map::GridMap m_gridMap;

    //! ROS subscribers
    ros::Subscriber m_elevationMapSubscriber;

    //! ROS publishers
    ros::Publisher m_gridMapPublisher;

    //! Thread for grid map post processing
    std::thread m_thread;
};