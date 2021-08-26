/*
 * vec2d.hpp
 *
 *  Created on: Aug 25, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

/**
  * Coordinate structure
  */
struct Vec2D
{
    //! X position
    int x;

    //! Y position
    int y;

    //! Equality operator for the struct
    inline bool operator == (const Vec2D& coordinates_) const
    {
        return (x == coordinates_.x && y == coordinates_.y);
    }

    //! Add operator for the struct
    inline Vec2D operator + (const Vec2D& rhs_) const
    {
        return{ x + rhs_.x, y + rhs_.y };
    }
};