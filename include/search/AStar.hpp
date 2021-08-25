/*
 * AStar.hpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

// C++ general
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>

// Robot model
#include <model.hpp>

// Structs
#include <structs/vec2d.hpp>
#include <structs/action.hpp>

// Config
#include <config.hpp>

namespace AStar
{
    /**
     * Basic structure for search nodes
     */
    struct Node
    {
        //! Parent node
        Node *parent;

        //! Node (x,y) position in the grid
        Vec2D coordinates;

        //! Costs
        unsigned int G, H;

        //! Constructor
        explicit Node(Vec2D coord_, Node *parent_ = nullptr);

        //! Routine to get node's total cost
        unsigned int getScore() const;
    };

    class Search
    {
    public:
        /**
         * Constructor.
         */
        explicit Search();

        /**
         * Destructor.
         */
        virtual ~Search();

        /**
         * Find path from source to target
         * in a given height map.
         *
         * @param source_
         * @param target_
         * @return sequence of 2D points (grid cells indexes)
         */
        std::vector<Vec2D> findPath(Vec2D source_, Vec2D target_);
    private:
        //! Grid map size
        Vec2D worldSize;

        //! Number of available actions
        unsigned int numberOfActions;

        //! Allowed actions in the search
        std::vector<Action> actions;

        //! Velocities
        std::vector<double> velocities;

        //! Heuristic function to be used
        std::function<unsigned int(Vec2D, Vec2D)> heuristic;

        //! Robot model
        Model m_model;

        /**
         * Set grid size.
         *
         * @param worldSize_
         */
        void setWorldSize(Vec2D worldSize_);

        /**
         * Sets whether the search uses a 4
         * or 8 neighbor expansion of the nodes.
         *
         * @param enable_
         */
        void setDiagonalMovement(bool enable_);

        /**
         * Check if expanded coordinate is valid
         * with respect to the grid bounds as well
         * as if the height is acceptable.
         *
         * @param coordinates_
         * @return if grid cell without bounds
         */
        bool detectCollision(Vec2D coordinates_) const;

        /**
         * Release the node pointers within collection.
         *
         * @param nodes_
         */
        void releaseNodes(std::vector<Node*>& nodes_);

        /**
         * Find if given node is in a vector (open/closed set).
         *
         * @param nodes_
         * @param coordinates_
         * @return the requested node or a nullptr
         */
        Node* findNodeOnList(std::vector<Node*>& nodes_, Vec2D coordinates_);

        /**
         * Sets heuristic to be used for the H cost.
         *
         * @param heuristic_
         */
        void setHeuristic(const std::function<unsigned int(Vec2D, Vec2D)>& heuristic_);
    };

    class Heuristic
    {
        /**
         * A* Heuristic routine that returns the
         * difference between two coordinate points.
         *
         * @param source_
         * @param target_
         * @return points' coordinate difference
         */
        static Vec2D getDelta(Vec2D source_, Vec2D target_);

    public:
        /**
         * A* Heuristic class routine that computes
         * the manhattan distance between two points.
         *
         * @param source_
         * @param target_
         * @return manhattan distance
         */
        static unsigned int manhattan(Vec2D source_, Vec2D target_);

        /**
         * A* Heuristic class routine that computes
         * the euclidean distance between two points.
         *
         * @param source_
         * @param target_
         * @return euclidean distance
         */
        static unsigned int euclidean(Vec2D source_, Vec2D target_);

        /**
         * A* Heuristic class routine that computes
         * the octagonal distance between two points.
         *
         * @param source_
         * @param target_
         * @return octagonal distance
         */
        static unsigned int octagonal(Vec2D source_, Vec2D target_);
    };
}