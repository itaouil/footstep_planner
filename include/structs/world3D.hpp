/*
 * world3D.hpp
 *
 *  Created on: Aug 26, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

/**
  * World (continuous) coordinates
  */
struct World3D
{
    //! X position in the world
    double x;

    //! Y position in the world
    double y;

    //! Z position in the world
    double z;

    //! Robot rotation w.r.t to map frame
    tf2::Quaternion q;
};