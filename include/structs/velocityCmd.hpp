/*
 * velocityCmd.hpp
 *
 *  Created on: Aug 25, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

/**
 * Velocity command structure
 */
struct VelocityCmd
{
    //! Forward velocity
    double x;

    //! Side velocity
    double y;

    //! Rotational velocity
    double theta;

    //! Struct comparator
    inline bool operator < ( const VelocityCmd &rhs ) const
    {
        return ( x < rhs.x || (x == rhs.x && y < rhs.y) || (x == rhs.x && y == rhs.y && theta < rhs.theta));
    }
};