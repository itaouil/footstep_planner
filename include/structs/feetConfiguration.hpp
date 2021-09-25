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
    //! Which set of feet is swinging
    bool fr_rl_swinging;

    //! FL foot
    World2D flCoM;
    World2D flMap;

    //! FR foot
    World2D frCoM;
    World2D frMap;

    //! RL foot
    World2D rlCoM;
    World2D rlMap;

    //! RR foot
    World2D rrCoM;
    World2D rrMap;
};