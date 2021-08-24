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

}

/**
 * Destructor
 */
Model::~Model() {

}

/**
 * Computes next CoM (Centre of Mass) by
 * integrating over a time stamp and applied
 * velocity.
 *
 * @param p_currentPosition
 * @param p_action
 * @param p_velocity
 * @return next CoM 2D position
 */
AStar::Vec2D Model::propagateCoM(AStar::Vec2D &p_futurePositon,
                                 const AStar::Vec2D &p_currentPosition,
                                 const AStar::Action &p_action,
                                 double p_velocity)
{
    return AStar::Vec2D();
}

