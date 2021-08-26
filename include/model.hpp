/*
 * model.hpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++
#include <map>

// ROS
#include <ros/ros.h>

// Structs
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
#include <structs/world2D.hpp>
#include <structs/velocityCmd.hpp>
#include <structs/displacement.hpp>

class Model
{
public:
    /**
     * Constructor.
     */
    explicit Model();

    /**
     * Destructor.
     */
    virtual ~Model();

    /**
     * Compute next CoM (Centre of Mass) given
     * the action and velocity applied.
     *
     * @param p_velocity
     * @param p_action
     * @param p_currentCoM
     * @param p_propagateCoM
     */
    void propagateCoM(double p_velocity, const Action &p_action, World2D p_currentCoM, World2D &p_propagateCoM);
private:
    //! Dictionary containing displacements for each velocity
    std::map<VelocityCmd, Displacement> m_displacementMap;
};