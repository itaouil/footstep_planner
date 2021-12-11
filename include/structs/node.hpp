/*
 * node.hpp
 *
 *  Created on: Aug 24, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// Structs
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
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

    //! Velocity applied to reach state
    double velocity;

    //! Costs (global and heuristic)
    unsigned int G, H;

    //! (x,y) (discrete) coordinates in the grid
    Vec2D gridCoordinates;

    //! (x,y) (continuous) coordinates in the world
    World3D worldCoordinates;

    //! Feet configuration of the state (CoM frame)
    FeetConfiguration feetConfiguration;

    /**
     * Struct constructor.
     *
     * @param p_gridCoordinates
     * @param p_worldCoordinates
     * @param p_parent
     */
    explicit Node(Action p_action,
                  Vec2D p_gridCoordinates,
                  World3D p_worldCoordinates,
                  FeetConfiguration feetConfiguration,
                  Node *p_parent = nullptr):
            parent(p_parent),
            action(p_action),
            G(0),
            H(0),
            gridCoordinates(p_gridCoordinates),
            worldCoordinates(p_worldCoordinates),
            feetConfiguration(feetConfiguration)
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