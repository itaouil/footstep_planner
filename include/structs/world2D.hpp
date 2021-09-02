/*
 * world2D.hpp
 *
 *  Created on: Aug 26, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

/**
  * World (continuous) coordinates
  */
struct World2D
{
    //! X position in the world
    double x{};

    //! Y position in the world
    double y{};

    //! Robot rotation w.r.t to map frame
    tf2::Quaternion q;
};