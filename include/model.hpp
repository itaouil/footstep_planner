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
#include <structs/vec2d.hpp>
#include <structs/action.hpp>
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
     * @param p_currentPosition
     * @param p_action
     * @param p_velocity
     */
    void propagateCoM(Vec2D &p_futurePosition,
                      const Vec2D &p_currentPosition,
                      const Action &p_action,
                      double p_velocity);
private:
    //! Dictionary containing displacements for each velocity
    std::map<VelocityCmd, Displacement> m_displacementMap;
};