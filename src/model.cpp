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
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.1,0,0}, {0.029, 0.0016})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.1,0,0}, {-0.037, 0.0012})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.1}, {0.005, 0.0035})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.1}, {0.0039, 0.004})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.1,0}, {0.0052, -0.0064})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.1,0}, {0.0046, 0.0036})); //LEFT

    // 0.15 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.15,0,0}, {0.044, 0.0021})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.15,0,0}, {-0.053, 0.0014})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.15}, {0.0071, 0.0032})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.15}, {0.0061, 0.0041})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.15,0}, {0.0062, -0.010})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.15,0}, {0.0051, 0.006})); //LEFT

    // 0.2 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.2,0,0}, {0.061, 0.002})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.2,0,0}, {-0.07, 0.0012})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.2}, {0.0093, 0.0027})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.2}, {0.0082, 0.0045})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.2,0}, {0.0068, -0.014})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.2,0}, {0.0052, 0.0067})); //LEFT

    // 0.25 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.25,0,0}, {0.077, 0.003})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.25,0,0}, {-0.085, 0.0013})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.25}, {0.011, 0.0025})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.25}, {0.010, 0.005})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.25,0}, {0.0076, -0.018})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.25,0}, {0.005, 0.012})); //LEFT

    // 0.3 velocity displacement
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0.3,0,0}, {0.094, 0.003})); //FWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({-0.3,0,0}, {-0.102, 0.001})); //BWD
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,0.3}, {0.013, 0.003})); //COUNT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0,-0.3}, {0.012, 0.005})); //CLOCK
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,-0.3,0}, {0.0077,  -0.018})); //RIGHT
    m_displacementMap.insert(std::pair<VelocityCmd, Displacement>({0,0.3,0}, {0.0049, 0.012})); //LEFT
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
 * @param p_currentPosition
 * @param p_action
 * @param p_velocity
 */
void Model::propagateCoM(Vec2D &p_futurePosition,
                         const Vec2D &p_currentPosition,
                         const Action &p_action,
                         double p_velocity)
{
    // Create velocity command key
    VelocityCmd l_velocityCmdKey{p_action.x * p_velocity, p_action.y * p_velocity, p_action.theta * p_velocity};

    // Get associated key displacement
    Displacement l_velocityCmdDisplacement = m_displacementMap[l_velocityCmdKey];

    // Compute new CoM position
    p_futurePosition.x = p_currentPosition.x + l_velocityCmdDisplacement.x;
    p_futurePosition.y = p_currentPosition.y + l_velocityCmdDisplacement.y;

    ROS_INFO_STREAM("Propagate CoM. Prev position: " << p_currentPosition.x << ", " << p_currentPosition.y);
    ROS_INFO_STREAM("Propagate CoM. Future position: " << p_futurePosition.x << ", " << p_futurePosition.y);
}
