/*
 * action.hpp
 *
 *  Created on: Aug 25, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

/**
  * Action structure
  */
struct Action
{
    //! If forward velocity is enabled
    int x;

    //! If side velocity is enabled
    int y;

    //! If Rotational velocity is enabled
    int theta;

    //! Inequality operator for the struct
    inline bool operator != (const Action& p_action) const
    {
        return (x != p_action.x || y != p_action.y || theta != p_action.theta);
    }

    //! Equality operator for the struct
    inline bool operator == (const Action& p_action) const
    {
        return (x == p_action.x && y == p_action.y && theta == p_action.theta);
    }
};
