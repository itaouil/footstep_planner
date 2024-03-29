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
    float velocity;

    //! Costs (global and heuristic)
    float G, H;

    //! (x,y) (discrete) coordinates in the grid
    Vec2D gridCoordinates;

    //! (x,y) (continuous) coordinates in the world
    World3D worldCoordinates;

    //! Feet configuration of the state (CoM frame)
    FeetConfiguration feetConfiguration;

    //! Number of sequences till this node
    uint sequence;

    /**
     * Struct constructor.
     *
     * @param p_gridCoordinates
     * @param p_worldCoordinates
     * @param p_parent
     */
    explicit Node(uint sequence,
                  Action p_action,
                  Vec2D p_gridCoordinates,
                  World3D p_worldCoordinates,
                  FeetConfiguration feetConfiguration,
                  Node *p_parent = nullptr
                  ):
            sequence(sequence),
            parent(p_parent),
            action(p_action),
            G(0.0),
            H(0.0),
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
    inline float getScore() const
    {
        return G + H;
    }
};