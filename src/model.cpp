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
Model::Model() {
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
                         World2D &p_propagateCoM)
{
    // Create velocity command key
    VelocityCmd l_velocityCmdKey{p_action.x * p_velocity, p_action.y * p_velocity, p_action.theta * p_velocity};

    // Get associated key displacement
    Displacement l_velocityCmdDisplacement = m_displacementMap[l_velocityCmdKey];

    ROS_INFO_STREAM("Velocity cmd key: " << l_velocityCmdKey.x << ", " << l_velocityCmdKey.y << ", " << l_velocityCmdKey.theta);
    ROS_INFO_STREAM("Displacement: " << l_velocityCmdDisplacement.x << ", " << l_velocityCmdDisplacement.y);

    // Compute new CoM position
    p_propagateCoM.x = p_currentCoM.x + l_velocityCmdDisplacement.x;
    p_propagateCoM.y = p_currentCoM.y + l_velocityCmdDisplacement.y;

//    ROS_INFO_STREAM("Prev CoM: " << p_currentWorldX << ", " << p_currentWorldY);
//    ROS_INFO_STREAM("Propagated CoM: " << p_propagatedWorldX << ", " << p_propagatedWorldY);
}
