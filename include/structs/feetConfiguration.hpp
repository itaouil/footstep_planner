/*
 * feetConfiguration.hpp
 *
 *  Created on: Sep 02, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

#include <structs/world3D.hpp>

/**
  * World (continuous) coordinates
  * of the robot feet w.r.t the CoM.
  */
struct FeetConfiguration
{
    //! Which set of feet is swinging
    bool fr_rl_swinging;

    //! FL foot
    World3D flCoM;
    World3D flMap;

    //! FR foot
    World3D frCoM;
    World3D frMap;

    //! RL foot
    World3D rlCoM;
    World3D rlMap;

    //! RR foot
    World3D rrCoM;
    World3D rrMap;
};