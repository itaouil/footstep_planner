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
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
#include <structs/world2D.hpp>

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

        //! (x,y) (discrete) coordinates in the grid
        Vec2D gridCoordinates;

        //! (x,y) (continuous) coordinates in the world
        World2D worldCoordinates;

        //! Costs
        unsigned int G, H;

        //! Constructor
        explicit Node(Vec2D coord_, World2D world_, Node *parent_ = nullptr);

        //! Routine to get node's total cost
        unsigned int getScore() const;
    };

    /**
     * Convert from grid coordinates to world coordinates.
     *
     * @param p_originX
     * @param p_originY
     * @param p_gridCoordinates
     * @param p_worldCoordinates
     */
    void gridToWorld(double p_originX, double p_originY, Vec2D p_gridCoordinates, World2D &p_worldCoordinates);

    /**
     * Convert from world coordinates to grid coordinates.
     *
     * @param p_originX
     * @param p_originY
     * @param p_worldCoordinates
     * @param p_gridCoordinates
     * @return if conversion is successful
     */
    bool worldToGrid(double p_originX, double p_originY, World2D p_worldCoordinates, Vec2D &p_gridCoordinates);

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
         * Sets grid origin in static reference frame.
         *
         * @param p_originX
         * @param origin_y
         */
        void setGridOrigin(double p_originX, double p_originY);

        /**
         * Find path from source to target
         * in a given height map.
         *
         * @param source_
         * @param target_
         * @return sequence of 2D points (world coordinates)
         */
        std::vector<World2D> findPath(Vec2D source_, Vec2D target_);
    private:
        //! Robot model
        Model m_model;

        //! Grid map size
        Vec2D worldSize;

        //! Grid map origin
        double m_gridOriginX;
        double m_gridOriginY;

        //! Allowed actions in the search
        std::vector<Action> actions;

        //! Number of available actions
        unsigned int numberOfActions;

        //! Velocities
        std::vector<double> velocities;

        //! Heuristic function to be used
        std::function<unsigned int(Vec2D, Vec2D)> heuristic;

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