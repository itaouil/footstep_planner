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
    // Processed elevation map publisher
    m_elevationMapProcessedPublisher = m_nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);

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
void ElevationMapProcessor::elevationMapCallback(const grid_map_msgs::GridMap &p_elevationMapMsg)
{
    // Convert grid_map_msgs to grid_map
    auto t0 = high_resolution_clock::now();
    grid_map::GridMap l_elevationMap;
    if (!grid_map::GridMapRosConverter::fromMessage(p_elevationMapMsg, l_elevationMap))
    {
        ROS_ERROR("ElevationMapProcessor: Could not convert grid_map_msgs to grid_map.");
    }
    auto t1 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to convert grid_map_msgs to grid_map: " << duration_cast<microseconds>(t1 - t0).count() << "micros");

    // Convert elevation map to OpenCV
    cv::Mat l_elevationMapImage;
    if (!grid_map::GridMapCvConverter::toImage<unsigned char, 1>(l_elevationMap,
                                                                 "elevation_inpainted",
                                                                 CV_8UC1,
                                                                 l_elevationMap.get("elevation_inpainted").minCoeffOfFinites(),
                                                                 l_elevationMap.get("elevation_inpainted").maxCoeffOfFinites(),
                                                                 l_elevationMapImage))
    {
        ROS_ERROR("ElevationMapProcessor: Could not convert grid_map to cv::Mat.");
    }
    auto t2 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to convert grid_map to opencv mat: " << duration_cast<microseconds>(t2 - t1).count() << "micros");

    // Blur elevation map image to reduce noise
    cv::Mat l_elevationMapBlurred;
    cv::GaussianBlur(l_elevationMapImage, l_elevationMapBlurred, cv::Size(3,3), 0, 0);
    auto t3 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to blur image: " << duration_cast<microseconds>(t3 - t2).count() << "micros");

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
    auto t4 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to compute x and y gradients: " << duration_cast<microseconds>(t4 - t3).count() << "micros");

    // Build overall filter result by combining the previous results
    cv::Mat l_sobelMagnitude;
    cv::magnitude(l_sobelX, l_sobelY, l_sobelMagnitude);
    auto t5 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to compute gradients magnitude: " << duration_cast<microseconds>(t5 - t4).count() << "micros");

    // Set costmap traversability based on computed gradients
    cv::Mat l_costmap;
    cv::threshold(l_sobelMagnitude, l_costmap, GRADIENT_THRESHOLD, 1, cv::THRESH_BINARY);
    auto t6 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to compute costmap: " << duration_cast<microseconds>(t6 - t5).count() << "micros");

    // Flip costmap around the horizontal
    // axis and subsequently rotate it to
    // obtain it in the same orientation as
    // the original elevation map
    cv::Mat l_costmapFlipped;
    cv::Mat l_costmapRotated;
    cv::flip(l_costmap, l_costmapFlipped, 1);
    cv::rotate(l_costmapFlipped, l_costmapRotated, cv::ROTATE_90_CLOCKWISE);
    auto t7 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to flip and rotate costmap: " << duration_cast<microseconds>(t7 - t6).count() << "micros");

    // Add new costmaps and the relative
    // elevation map eigen matrix
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);

        if (m_footCostmaps.size() > 4)
        {
            m_footCostmaps.pop();
            m_elevationMaps.pop();
        }

        m_footCostmaps.push(l_costmap);
        m_elevationMaps.push(l_elevationMap.get("elevation_inpainted"));
    }

    auto t8 = high_resolution_clock::now();
//    ROS_INFO_STREAM("Time took to push costmap and clean queue: " << duration_cast<microseconds>(t8 - t7).count() << "micros");
//    ROS_INFO_STREAM("Overall time took for costmap computation: " << duration_cast<microseconds>(t8 - t0).count() << "micros");

    if (PUBLISH)
    {
        std::vector<float> l_data;
        if (l_costmapRotated.isContinuous()) {
            l_data.assign((float*)l_costmapRotated.data, (float*)l_costmapRotated.data + l_costmapRotated.total()*l_costmapRotated.channels());
        } else {
            for (int i = 0; i < l_sobelMagnitude.rows; ++i) {
                l_data.insert(l_data.end(), l_costmapRotated.ptr<float>(i), l_costmapRotated.ptr<float>(i)+l_costmapRotated.cols*l_costmapRotated.channels());
            }
        }

        // Create occupancy grid from computed gradients
        nav_msgs::OccupancyGrid l_occupancyGrid;
        l_occupancyGrid.header = p_elevationMapMsg.info.header;
        l_occupancyGrid.info.resolution = static_cast<float>(p_elevationMapMsg.info.resolution);
        l_occupancyGrid.info.width = static_cast<int>(p_elevationMapMsg.info.length_x / l_occupancyGrid.info.resolution);
        l_occupancyGrid.info.height = static_cast<int>(p_elevationMapMsg.info.length_y / l_occupancyGrid.info.resolution);
        l_occupancyGrid.info.origin.position.x = p_elevationMapMsg.info.pose.position.x - p_elevationMapMsg.info.length_x/2;
        l_occupancyGrid.info.origin.position.y = p_elevationMapMsg.info.pose.position.y - p_elevationMapMsg.info.length_y/2;
        l_occupancyGrid.info.origin.position.z = 0.0;

        // Populate occupancy grid (for visualization purposes only)
        for (auto i : l_data) {
            if (static_cast<int>(i) > 0)
                l_occupancyGrid.data.push_back(100);
            else
                l_occupancyGrid.data.push_back(0);
        }

        // Publish occupancy grid
        m_elevationMapProcessedPublisher.publish(l_occupancyGrid);
    }
}

/**
 * Obtain latest processed costmap.
 */
std::tuple<cv::Mat, grid_map::Matrix> ElevationMapProcessor::getCostmaps()
{
    {
        std::lock_guard<std::mutex> l_lockGuard(m_mutex);
        return std::make_tuple(m_footCostmaps.back(), m_elevationMaps.back());
    }
}