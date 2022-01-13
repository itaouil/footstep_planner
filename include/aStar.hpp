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

// Elevation map processor
#include <elevationMapProcessor.hpp>

// Structs
#include <structs/node.hpp>
#include <structs/vec2D.hpp>
#include <structs/action.hpp>
#include <structs/world3D.hpp>

// Config
#include <config.hpp>

namespace AStar {
    /**
      * Obtain yaw angle from
      * respective quaternion rotation.
      *
      * @param p_quaternion
      * @return yaw angle from quaternion
      */
    double getYawFromQuaternion(const tf2::Quaternion &p_quaternion);

    class Search {
    public:
        /**
         * Constructor.
         */
        explicit Search(ros::NodeHandle &p_nh);

        /**
         * Destructor.
         */
        virtual ~Search();

        /**
         * Find path from source to target
         * in a given height map.
         *
         * @param p_sourceWorldCoordinates
         * @param p_targetWorldCoordinates
         * @param p_sourceFeetConfiguration
         * @return sequence of 2D points (world coordinates)
         */
        void findPath(const Action &p_initialAction,
                      const double &p_initialVelocity,
                      const World3D &p_sourceWorldCoordinates,
                      const World3D &p_targetWorldCoordinates,
                      const FeetConfiguration &p_sourceFeetConfiguration,
                      std::vector<Node> &p_path);

    private:
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
        void releaseNodes(std::vector<Node *> &p_nodes);

        /**
         * Find if given node is in a vector (open/closed set).
         *
         * @param p_nodes
         * @param p_action
         * @param p_velocity
         * @param p_gridCoordinates
         * @param p_quaternion
         * @return the requested node or a nullptr
         */
        Node *findNodeOnList(const std::vector<Node *> &p_nodes,
                             const Action &p_action,
                             double p_velocity,
                             const Vec2D &p_gridCoordinates,
                             const tf2::Quaternion &p_quaternion);

        /**
         * Sets heuristic to be used for the H cost.
         *
         * @param p_heuristic
         */
        void setHeuristic(const std::function<unsigned int(Node, Node)> &p_heuristic);

        /**
         * Check if current node coordinates
         * are within the target tolerance
         * distance.
         *
         * @param p_nodeGridCoordinates
         * @param p_targetGridCoordinates
         * @param p_nodeQuaternion
         * @param p_targetQuaternion
         * @return if coordinates within distance tolerance
         */
        bool targetReached(const Vec2D &p_nodeGridCoordinates,
                           const Vec2D &p_targetGridCoordinates,
                           const tf2::Quaternion &p_nodeQuaternion,
                           const tf2::Quaternion &p_targetQuaternion);

        /**
         * Convert from world coordinates to grid coordinates.
         *
         * @param p_worldCoordinates
         * @param p_gridCoordinates
         * @return if conversion is successful
         */
        bool worldToGrid(const World3D &p_worldCoordinates, Vec2D &p_gridCoordinates);

        /**
         * Transform feet configuration
         * from CoM frame to map frame.
         *
         * @param p_newCoMWorldCoordinates
         * @param p_newFeetConfigurationCoM
         * @param p_newFeetConfigurationMap
         */
        void transformCoMFeetConfigurationToMap(const World3D &p_newCoMWorldCoordinates,
                                                const FeetConfiguration &p_newFeetConfigurationCoM,
                                                FeetConfiguration &p_newFeetConfigurationMap);

        //! Robot model
        Model m_model;

        //! First search or not
        bool m_firstSearch;

        //! Footsteps validated
        unsigned int m_validFootstepsFound;

        //! TF2 buffer and listener
        tf2_ros::Buffer m_buffer;
        tf2_ros::TransformListener m_listener;

        //! Elevation map parameters
        double m_elevationMapGridOriginX{};
        double m_elevationMapGridOriginY{};
        double m_elevationMapGridResolution{};
        unsigned int m_elevationMapGridSizeX{};
        unsigned int m_elevationMapGridSizeY{};

        //! Elevation map processor (collisions, foothold validity)
        ElevationMapProcessor m_elevationMapProcessor;

        //! Allowed actions in the search
        std::vector<Action> m_actions;

        //! Number of available actions
        unsigned int m_numberOfActions{};

        //! Velocities
        std::vector<double> m_velocities;

        //! Heuristic function to be used
        std::function<unsigned int(Node, Node)> m_heuristic;

        //! Idle feet configuration
        FeetConfiguration m_idleFeetConfiguration;

        //! Robot origin w.r.t to map
        World3D m_robotOriginMapFrame;
    };

    class Heuristic {
    public:
        /**
         * A* Heuristic routine that returns the
         * difference between two coordinate points.
         *
         * @param p_sourceWorldCoordinates
         * @param p_targetWorldCoordinates
         * @return points' coordinate difference
         */
        static World3D
        getDistanceDelta(const World3D &p_sourceWorldCoordinates, const World3D &p_targetWorldCoordinates);

        /**
         * A* Heuristic class routine that computes
         * theta difference between source and target
         *
         * @param p_sourceWorldCoordinates
         * @param p_targetWorldCoordinates
         * @return theta distance
         */
        static double getHeadingDelta(const World3D &p_sourceWorldCoordinates, const World3D &p_targetWorldCoordinates);

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