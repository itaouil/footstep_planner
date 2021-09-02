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
#include <structs/node.hpp>
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
#include <structs/world2D.hpp>

// Config
#include <config.hpp>

namespace AStar
{
    /**
     * Obtain yaw angle from
     * respective quaternion rotation.
     *
     * @param p_quaternion
     * @return yaw angle from quaternion
     */
    double getYawFromQuaternion(const tf2::Quaternion &p_quaternion);

    /**
     * Convert from grid coordinates to world coordinates.
     *
     * @param p_originX
     * @param p_originY
     * @param p_gridCoordinates
     * @param p_worldCoordinates
     */
    void gridToWorld(double p_originX, double p_originY, const Vec2D &p_gridCoordinates, World2D &p_worldCoordinates);

    /**
     * Convert from world coordinates to grid coordinates.
     *
     * @param p_originX
     * @param p_originY
     * @param p_worldCoordinates
     * @param p_gridCoordinates
     * @return if conversion is successful
     */
    bool worldToGrid(double p_originX, double p_originY, const World2D &p_worldCoordinates, Vec2D &p_gridCoordinates);

    class Search
    {
    public:
        /**
         * Constructor.
         */
        explicit Search(ros::NodeHandle& p_nh);

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
         * @param p_sourceWorldCoordinates
         * @param p_targetWorldCoordinates
         * @param p_sourceFeetConfiguration
         * @return sequence of 2D points (world coordinates)
         */
        std::vector<Node> findPath(const World2D &p_sourceWorldCoordinates,
                                   const World2D &p_targetWorldCoordinates,
                                   const FeetConfiguration &p_sourceFeetConfiguration);
    private:
        /**
         * Set grid map size.
         *
         * @param p_dimensionX
         * @param p_dimensionY
         */
        void setGridSize(unsigned int p_dimensionX, unsigned int p_dimensionY);

        /**
         * Sets whether the search uses a 4
         * or 8 neighbor expansion of the nodes.
         *
         * @param p_enable
         */
        void setDiagonalMovement(bool p_enable);

        /**
         * Check if expanded coordinate is valid
         * with respect to the grid bounds as well
         * as if the height is acceptable.
         *
         * @param p_gridCoordinates
         * @return if grid cell without bounds
         */
        bool detectCollision(const Vec2D &p_gridCoordinates) const;

        /**
         * Release the node pointers within collection.
         *
         * @param p_nodes
         */
        void releaseNodes(std::vector<Node*>& p_nodes);

        /**
         * Find if given node is in a vector (open/closed set).
         *
         * @param p_nodes
         * @param p_gridCoordinates
         * @param p_quaternion
         * @return the requested node or a nullptr
         */
        Node* findNodeOnList(const std::vector<Node*>& p_nodes,
                             const Vec2D &p_gridCoordinates,
                             const tf2::Quaternion &p_quaternion);

        /**
         * Sets heuristic to be used for the H cost.
         *
         * @param heuristic_
         */
        void setHeuristic(const std::function<unsigned int(Node, Node)>& heuristic_);

        //! Robot model
        Model m_model;

        //! Grid map size
        Vec2D m_gridSize;

        //! Grid map origin
        double m_gridOriginX;
        double m_gridOriginY;

        //! Allowed actions in the search
        std::vector<Action> m_actions;

        //! Number of available actions
        unsigned int m_numberOfActions;

        //! Velocities
        std::vector<double> m_velocities;

        //! Heuristic function to be used
        std::function<unsigned int(Node, Node)> m_heuristic;
    };

    class Heuristic
    {
    public:
        /**
         * A* Heuristic routine that returns the
         * difference between two coordinate points.
         *
         * @param p_sourceGridCoordinates
         * @param p_targetGridCoordinates
         * @return points' coordinate difference
         */
        static Vec2D getDistanceDelta(const Vec2D &p_sourceGridCoordinates, const Vec2D &p_targetGridCoordinates);

        /**
         * A* Heuristic class routine that computes
         * theta difference between source and target
         *
         * @param p_sourceWorldCoordinates
         * @param p_targetWorldCoordinates
         * @return theta distance
         */
        static double getHeadingDelta(const World2D &p_sourceWorldCoordinates, const World2D &p_targetWorldCoordinates);

        /**
         * A* Heuristic class routine that computes
         * the manhattan distance between two points.
         *
         * @param p_sourceNode
         * @param p_targetNode
         * @return manhattan distance
         */
        static unsigned int manhattan(const Node &p_sourceNode, const Node &p_targetNode);

        /**
         * A* Heuristic class routine that computes
         * the euclidean distance between two points.
         *
         * @param p_sourceNode
         * @param p_targetNode
         * @return euclidean distance
         */
        static unsigned int euclidean(const Node &p_sourceNode, const Node &p_targetNode);

        /**
         * A* Heuristic class routine that computes
         * the octagonal distance between two points.
         *
         * @param p_sourceNode
         * @param p_targetNode
         * @return octagonal distance
         */
        static unsigned int octagonal(const Node &p_sourceNode, const Node &p_targetNode);
    };
}