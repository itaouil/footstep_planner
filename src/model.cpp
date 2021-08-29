/*
 * model.cpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include <model.hpp>

/**
 * Constructor
 */
Model::Model()
{
    // 0.1 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.1,0,0}, {0.029, 0.0035})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.1,0,0}, {-0.037, 0.0035})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.1}, {0.0049, 0.005})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.1}, {0.0039, 0.0069})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.1,0}, {0.0052, -0.035})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.1,0}, {0.0046, 0.033})); //LEFT

    // 0.15 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.15,0,0}, {0.044, 0.0038})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.15,0,0}, {-0.053, 0.0041})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.15}, {0.0071, 0.0089})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.15}, {0.0061, 0.010})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.15,0}, {0.0062, -0.052})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.15,0}, {0.0051, 0.052})); //LEFT

    // 0.2 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.2,0,0}, {0.061, 0.0038})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.2,0,0}, {-0.07, 0.0046})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.2}, {0.0093, 0.012})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.2}, {0.0082, 0.013})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.2,0}, {0.0068, -0.069})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.2,0}, {0.0052, 0.070})); //LEFT

    // 0.25 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.25,0,0}, {0.077, 0.0042})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.25,0,0}, {-0.085, 0.0046})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.25}, {0.011, 0.015})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.25}, {0.010, 0.016})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.25,0}, {0.0076, -0.085})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.25,0}, {0.005, 0.086})); //LEFT

    // 0.3 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.3,0,0}, {0.094, 0.0047})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.3,0,0}, {-0.102, 0.005})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.3}, {0.013, 0.018})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.3}, {0.012, 0.019})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.3,0}, {0.0077,  -0.086})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.3,0}, {0.0049, 0.087})); //LEFT
}

/**
 * Destructor
 */
Model::~Model() {

}

/**
 * Compute next CoM (Centre of Mass) given
 * the action and velocity applied.
 *
 * @param p_velocity
 * @param p_action
 * @param p_currentWorldX
 * @param p_currentWorldY
 * @param p_propagatedWorldX
 * @param p_propagatedWorldY
 */
void Model::propagateCoM(double p_velocity,
                         const Action &p_action,
                         World2D p_currentCoM,
                         World2D &p_propagatedCoM)
{
    // Create velocity command key
    VelocityCmd l_velocityCmdKey{p_action.x * p_velocity, p_action.y * p_velocity, p_action.theta * p_velocity};

    // Get associated command displacement
    Displacement l_velocityCmdDisplacement = m_displacementMap[l_velocityCmdKey];

    // Compute rotation displacement in radians
    double l_angularDisplacementRadians = l_velocityCmdKey.theta * TIMESTAMP;

    // Create current rotation transform
    geometry_msgs::TransformStamped l_rotationTransform;
    l_rotationTransform.header.stamp = ros::Time::now();
    l_rotationTransform.header.frame_id = HEIGHT_MAP_REFERENCE_FRAME;
    l_rotationTransform.transform.translation.x = 0;
    l_rotationTransform.transform.translation.y = 0;
    l_rotationTransform.transform.translation.z = 0;
    l_rotationTransform.transform.rotation.x = p_currentCoM.q.x;
    l_rotationTransform.transform.rotation.y = p_currentCoM.q.y;
    l_rotationTransform.transform.rotation.z = p_currentCoM.q.z;
    l_rotationTransform.transform.rotation.w = p_currentCoM.q.w;

    // Populate robot frame displacement vector
    geometry_msgs::PointStamped l_displacementRobotFrame;
    l_displacementRobotFrame.header.stamp = ros::Time::now();
    l_displacementRobotFrame.header.frame_id = ROBOT_REFERENCE_FRAME;
    l_displacementRobotFrame.point.x = l_velocityCmdDisplacement.x;
    l_displacementRobotFrame.point.y = l_velocityCmdDisplacement.y;
    l_displacementRobotFrame.point.z = 0;

    // Apply transform to obtain displacement
    // vector in the map reference frame
    geometry_msgs::PointStamped l_displacementMapFrame;
    tf2::doTransform(l_displacementRobotFrame, l_displacementMapFrame, l_rotationTransform);

    ROS_INFO_STREAM("Velocity cmd key: " << l_velocityCmdKey.x << ", " << l_velocityCmdKey.y << ", " << l_velocityCmdKey.theta);
    ROS_INFO_STREAM("Displacement: " << l_displacementMapFrame.point.x << ", " << l_displacementMapFrame.point.y);
    ROS_INFO_STREAM("Angular Displacement: " << l_angularDisplacementRadians << "\n");

    // Compute new CoM position
    p_propagatedCoM.x = p_currentCoM.x + l_displacementMapFrame.point.x;
    p_propagatedCoM.y = p_currentCoM.y + l_displacementMapFrame.point.y;
    p_propagatedCoM.q.x = p_currentCoM.q.x;
    p_propagatedCoM.q.y = p_currentCoM.q.y;
    p_propagatedCoM.q.z = p_currentCoM.q.z + l_angularDisplacementRadians;
    p_propagatedCoM.q.w += p_currentCoM.q.w;

//    ROS_INFO_STREAM("Prev CoM: " << p_currentWorldX << ", " << p_currentWorldY);
//    ROS_INFO_STREAM("Propagated CoM: " << p_propagatedWorldX << ", " << p_propagatedWorldY);
}
