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
};
