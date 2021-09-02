/*
 * node.hpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

#include <structs/feetConfiguration.hpp>

/**
 * Basic structure for A* nodes
 */
struct Node
{
    //! Parent node
    Node *parent;

    //! Action applied to reach state
    Action action;

    //! Costs (global and heuristic)
    unsigned int G, H;

    //! (x,y) (discrete) coordinates in the grid
    Vec2D gridCoordinates;

    //! (x,y) (continuous) coordinates in the world
    World2D worldCoordinates;

    //! Feet configuration of the state
    FeetConfiguration feetConfiguration;

    /**
     * Struct constructor.
     *
     * @param p_gridCoordinates
     * @param p_worldCoordinates
     * @param p_parent
     */
    explicit Node(Vec2D p_gridCoordinates, World2D p_worldCoordinates, Node *p_parent = nullptr):
        parent(p_parent),
        G(0),
        H(0),
        gridCoordinates(p_gridCoordinates),
        worldCoordinates(p_worldCoordinates)
    {}

    /**
     * Struct routine that returns
     * the total cost of the node.
     *
     * @return total node cost => H(s) + G(s)
     */
    inline unsigned int getScore() const
    {
        return G + H;
    }
};