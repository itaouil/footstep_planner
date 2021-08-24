/*
 * AStar.hpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>

// Config
#include <config.hpp>

namespace AStar
{
    /**
      * Actions structure
      */
    struct Action
    {
        //! Forward velocity
        double x;

        //! Side velocity
        double y;

        //! Rotational velocity
        double theta;
    };

    /**
     * Basic structure for grid indexing
     */
    struct Vec2i
    {
        //! X position in the grid map
        int x;

        //! Y position in the grid map
        int y;

        //! Equality operator for the struct
        bool operator == (const Vec2i& coordinates_) const;
    };

    /**
     * Basic structure for search nodes
     */
    struct Node
    {
        //! Parent node
        Node *parent;

        //! Node (x,y) position in the grid
        Vec2i coordinates;

        //! Costs
        unsigned int G, H;

        //! Constructor
        explicit Node(Vec2i coord_, Node *parent_ = nullptr);

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
        std::vector<Vec2i> findPath(Vec2i source_, Vec2i target_);
    private:
        //! Grid map size
        Vec2i worldSize;

        //! Considered neighbours
        unsigned int directions;

        //! Allowed direction in the search
        std::vector<Vec2i> direction;

        //! Allowed actions in the search
        std::vector<Action> actions;

        //! Heuristic function to be used
        std::function<unsigned int(Vec2i, Vec2i)> heuristic;

        /**
         * Set grid size.
         *
         * @param worldSize_
         */
        void setWorldSize(Vec2i worldSize_);

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
        bool detectCollision(Vec2i coordinates_);

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
        Node* findNodeOnList(std::vector<Node*>& nodes_, Vec2i coordinates_);

        /**
         * Sets heuristic to be used for the H cost.
         *
         * @param heuristic_
         */
        void setHeuristic(const std::function<unsigned int(Vec2i, Vec2i)>& heuristic_);
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
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        /**
         * A* Heuristic class routine that computes
         * the manhattan distance between two points.
         *
         * @param source_
         * @param target_
         * @return manhattan distance
         */
        static unsigned int manhattan(Vec2i source_, Vec2i target_);

        /**
         * A* Heuristic class routine that computes
         * the euclidean distance between two points.
         *
         * @param source_
         * @param target_
         * @return euclidean distance
         */
        static unsigned int euclidean(Vec2i source_, Vec2i target_);

        /**
         * A* Heuristic class routine that computes
         * the octagonal distance between two points.
         *
         * @param source_
         * @param target_
         * @return octagonal distance
         */
        static unsigned int octagonal(Vec2i source_, Vec2i target_);
    };
}

