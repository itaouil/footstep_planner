/*
 * world2D.hpp
 *
 *  Created on: Sep 02, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

#include <structs/world2D.hpp>

/**
  * World (continuous) coordinates
  * of the robot feet w.r.t the CoM.
  */
struct FeetConfiguration
{
    //! FL foot
    World2D fl;

    //! FR foot
    World2D fr;

    //! RL foot
    World2D rl;

    //! RR foot
    World2D rr;
};