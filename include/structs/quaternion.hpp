/*
 * quaternion.hpp
 *
 *  Created on: Aug 26, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

/**
  * Quaternion structure that holds
  * robot rotation in the map frame.
  */
struct Quaternion
{
    //! X component
    double x;

    //! Y component
    double y;

    //! Z component
    double z;

    //! W component
    double w;
};