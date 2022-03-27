/*
 * AStar.cpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "aStar.hpp"

std::ofstream m_fileStream;

/**
 * Obtain yaw angle (in degrees) from
 * respective quaternion rotation.
 *
 * @param p_quaternion
 * @return yaw angle from quaternion
 */
double AStar::getYawFromQuaternion(const tf2::Quaternion &p_quaternion) {
    double l_roll, l_pitch, l_yaw;
    tf2::Matrix3x3 l_rotationMatrix(p_quaternion);
    l_rotationMatrix.getRPY(l_roll, l_pitch, l_yaw);
    return ((l_yaw * 180) / M_PI);
}

/**
 * Convert from world coordinates to grid coordinates.
 *
 * @param p_worldCoordinates
 * @param p_gridCoordinates
 * @return if conversion is successful
 */
bool AStar::Search::worldToGrid(const World3D &p_worldCoordinates,
                                Vec2D &p_gridCoordinates) {
    // Compute transformation
    m_elevationMapProcessor.worldToGrid(p_worldCoordinates, p_gridCoordinates);
}

/**
 * A* search class constructor
 */
AStar::Search::Search(ros::NodeHandle &p_nh) :
        m_model(p_nh),
        m_firstSearch(true),
        m_validFootstepsFound(0),
        m_listener(m_buffer),
        m_elevationMapProcessor(p_nh) {
    // Set if diagonal movements are allowed
    setDiagonalMovement(SET_DIAGONAL_MOVEMENT);

    // Available actions
    m_actions = {
            {1, 0,  0}, // Forward
            {0, -1, 0}, // Right
            {0, 1,  0}, // Left
            {0, 0,  -1}, // Clockwise
            {0, 0,  1}, // Counterclockwise
            {1, 0,  -1}, // Forward + Counterclockwise
            {1, 0,  1} // Forward + Counterclockwise
    };

    // Available velocities
    m_velocities = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};

    // Open file stream file
    if (!m_fileStream.is_open()) {
        m_fileStream.open(
                "/home/itaouil/workspace/code/thesis_ws/src/footstep_planner/data/planner_results/parkour2/02.txt");
    }
}

/**
 * A* search class destructor
 */
AStar::Search::~Search() = default;

/**
 * Sets whether the search uses a 5
 * or 7 neighbor expansion for the nodes
 *
 * @param p_enable
 */
void AStar::Search::setDiagonalMovement(bool p_enable) {
    m_numberOfActions = (p_enable ? 7 : 1);
}

/**
 * Check if expanded coordinate is valid
 * with respect to the grid bounds as well
 * as if the height is acceptable.
 *
 * @param p_gridCoordinates
 * @return if grid cell without bounds
 */
bool AStar::Search::detectCollision(const Vec2D &p_gridCoordinates) const {
    //TODO: add height check
    if (p_gridCoordinates.x < 0 || p_gridCoordinates.x >= static_cast<int>(m_elevationMapGridSizeX) ||
        p_gridCoordinates.y < 0 || p_gridCoordinates.y >= static_cast<int>(m_elevationMapGridSizeY)) {
        // ROS_INFO("AStar: Collision detected.");
        // ROS_DEBUG_STREAM(p_gridCoordinates.x << ", " << m_elevationMapGridSizeX);
        // ROS_DEBUG_STREAM(p_gridCoordinates.y << ", " << m_elevationMapGridSizeY);
        return true;
    }
    return false;
}

/**
 * Release the node pointers within collection.
 *
 * @param p_nodes
 */
void AStar::Search::releaseNodes(std::vector<Node *> &p_nodes) {
    for (auto it = p_nodes.begin(); it != p_nodes.end();) {
        delete *it;
        it = p_nodes.erase(it);
    }
}

/**
 * Find if given node is in a vector (open/closed set).
 *
 * @param p_nodes
 * @param p_action
 * @param p_velocity
 * @param p_gridCoordinates
 * @param p_quaternion
 * @param p_angleCorrection
 * @return the requested node or a nullptr
 */
Node *AStar::Search::findNodeOnList(const std::vector<Node *> &p_nodes,
                                    const Action &p_action,
                                    const double p_velocity,
                                    const Vec2D &p_gridCoordinates,
                                    const tf2::Quaternion &p_quaternion) {
    for (auto &node: p_nodes) {
        // Check if the two nodes have the same state
        double l_yawDifference = std::abs(
                getYawFromQuaternion(node->worldCoordinates.q) - getYawFromQuaternion(p_quaternion));
        if (node->gridCoordinates == p_gridCoordinates &&
            node->action == p_action &&
            node->velocity == p_velocity &&
            l_yawDifference < 1) {
            ROS_DEBUG_STREAM("Action brought to already visited CoM. " << "\n");
            ROS_DEBUG_STREAM("Same node: " << node->gridCoordinates.x << ", "
                                           << node->gridCoordinates.y << ", "
                                           << p_gridCoordinates.x << ", "
                                           << p_gridCoordinates.y << ", "
                                           << p_action.x << ", " << p_action.y << ", " << p_action.theta << ", "
                                           << l_yawDifference << "\n");
            return node;
        }
    }

    return nullptr;
}

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
bool AStar::Search::targetReached(const Vec2D &p_nodeGridCoordinates,
                                  const Vec2D &p_targetGridCoordinates,
                                  const tf2::Quaternion &p_nodeQuaternion,
                                  const tf2::Quaternion &p_targetQuaternion) {
    double l_currentHeading = getYawFromQuaternion(p_nodeQuaternion);
    double l_targetHeading = getYawFromQuaternion(p_targetQuaternion);

    ROS_DEBUG_STREAM("Tolerance angle: " << l_currentHeading << ", " << l_targetHeading << ", "
                                         << (std::abs(l_currentHeading - l_targetHeading) <=
                                             ANGLE_DIFFERENCE_TOLERANCE));
    ROS_DEBUG_STREAM("Grid x tolerance: " << p_nodeGridCoordinates.x << ", " << p_targetGridCoordinates.x);
    ROS_DEBUG_STREAM("Grid y tolerance: " << p_nodeGridCoordinates.y << ", " << p_targetGridCoordinates.y << "\n");

    return ((std::abs(p_nodeGridCoordinates.x - p_targetGridCoordinates.x) <= 1) &&
            (std::abs(p_nodeGridCoordinates.y - p_targetGridCoordinates.y) <= 1) &&
            std::abs(l_currentHeading - l_targetHeading) <= ANGLE_DIFFERENCE_TOLERANCE);
}

/**
 * Transform feet configuration
 * from CoM frame to map frame.
 *
 * @param p_newCoMWorldCoordinates
 * @param p_newFeetConfiguration
 */
void AStar::Search::setFeetConfigurationMapFields(const World3D &p_newCoMWorldCoordinates,
                                                  FeetConfiguration &p_newFeetConfiguration) {
    p_newFeetConfiguration.flMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfiguration.flCoM.x;
    p_newFeetConfiguration.flMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfiguration.flCoM.y;
    p_newFeetConfiguration.frMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfiguration.frCoM.x;
    p_newFeetConfiguration.frMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfiguration.frCoM.y;
    p_newFeetConfiguration.rlMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfiguration.rlCoM.x;
    p_newFeetConfiguration.rlMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfiguration.rlCoM.y;
    p_newFeetConfiguration.rrMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfiguration.rrCoM.x;
    p_newFeetConfiguration.rrMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfiguration.rrCoM.y;
}

/**
 * Find path from source to target
 * in a given height map.
 *
 * @param p_initialAction
 * @param p_initialVelocity
 * @param p_sourceWorldCoordinates
 * @param p_targetWorldCoordinates
 * @param p_odomVelocityState
 * @param p_sourceFeetConfiguration
 * @param p_path
 */
void AStar::Search::findPath(const Action &p_initialAction,
                             const double &p_initialVelocity,
                             const World3D &p_sourceWorldCoordinates,
                             const World3D &p_targetWorldCoordinates,
                             const geometry_msgs::Twist &p_odomVelocityState,
                             const FeetConfiguration &p_sourceFeetConfiguration,
                             std::vector<Node> &p_path) {
    // Store initial idle feet configuration
    if (m_firstSearch) {
        m_firstSearch = false;
        m_idleFeetConfiguration = p_sourceFeetConfiguration;
    }

    // Reset/Set variables used during search
    m_validFootstepsFound = 0;
    unsigned int l_expandedNodes = 0;

    // Update elevation map parameters
    m_elevationMapProcessor.getElevationMapParameters(m_elevationMapGridOriginX,
                                                      m_elevationMapGridOriginY,
                                                      m_elevationMapGridResolution,
                                                      m_elevationMapGridSizeX,
                                                      m_elevationMapGridSizeY);

    // Convert source and target world coordinates to grid coordinates
    Vec2D l_sourceGridCoordinates{};
    Vec2D l_targetGridCoordinates{};
    worldToGrid(p_sourceWorldCoordinates, l_sourceGridCoordinates);
    worldToGrid(p_targetWorldCoordinates, l_targetGridCoordinates);

    // Create open and closed sets for the search process
    std::vector<Node *> l_openSet, l_closedSet;
    l_openSet.reserve(100);
    l_closedSet.reserve(100);

    // Current expanded node
    Node *l_currentNode = nullptr;

    // Push to initial open set the source node
    l_openSet.push_back(new Node(p_initialAction,
                                 l_sourceGridCoordinates,
                                 p_sourceWorldCoordinates,
                                 p_sourceFeetConfiguration));
    auto l_iterator = l_openSet.begin();
    auto l_initialNode = *l_iterator;
    l_initialNode->velocity = p_initialVelocity;

    // Search process
    bool l_lol = true;
    while (!l_openSet.empty()) {
        l_iterator = l_openSet.begin();
        l_currentNode = *l_iterator;

        // Find which node in the open
        // set to expand based on cost
        for (auto it = l_openSet.begin(); it != l_openSet.end(); it++) {
            auto l_iteratorNode = *it;
            if (l_iteratorNode->getScore() <= l_currentNode->getScore()) {
                l_currentNode = l_iteratorNode;
                l_iterator = it;
            }
        }

        if (l_lol) {
            float l_flDistance = 0;
            float l_frDistance = 0;
            float l_rlDistance = 0;
            float l_rrDistance = 0;
            Vec2D l_flGridPoses{};
            Vec2D l_frGridPoses{};
            Vec2D l_rlGridPoses{};
            Vec2D l_rrGridPoses{};
            worldToGrid(l_currentNode->feetConfiguration.flMap, l_flGridPoses);
            worldToGrid(l_currentNode->feetConfiguration.frMap, l_frGridPoses);
            worldToGrid(l_currentNode->feetConfiguration.rlMap, l_rlGridPoses);
            worldToGrid(l_currentNode->feetConfiguration.rrMap, l_rrGridPoses);
            m_elevationMapProcessor.validFootstep(l_flGridPoses.x,
                                                  l_flGridPoses.y,
                                                  l_flDistance);
            m_elevationMapProcessor.validFootstep(l_frGridPoses.x,
                                                  l_frGridPoses.y,
                                                  l_frDistance);
            m_elevationMapProcessor.validFootstep(l_rlGridPoses.x,
                                                  l_rlGridPoses.y,
                                                  l_rlDistance);
            m_elevationMapProcessor.validFootstep(l_rrGridPoses.x,
                                                  l_rrGridPoses.y,
                                                  l_rrDistance);

            // Log distances of the real feet
            m_fileStream << l_flDistance << ","
                         << l_frDistance << ", "
                         << l_rlDistance << ", "
                         << l_rrDistance << "\n";
            m_fileStream.flush();
            l_lol = false;
        }

        ROS_INFO_STREAM(
                "G value: " << l_currentNode->G << ", " << l_currentNode->H << ", " << l_currentNode->getScore());

        // If target was reached or already planned
        // for the footstep horizon stop any further
        // search
        if (m_validFootstepsFound == FOOTSTEP_HORIZON ||
            targetReached(l_currentNode->gridCoordinates, l_targetGridCoordinates,
                          l_currentNode->worldCoordinates.q, p_targetWorldCoordinates.q)) {
            ROS_INFO_STREAM("Search: Planning completed. " << m_validFootstepsFound);
            break;
        }

        // Push to closed set the currently expanded
        // node and delete its iterator from the open set
        l_closedSet.push_back(l_currentNode);
        l_openSet.erase(l_iterator);

        ROS_DEBUG_STREAM("Expanded node: " << l_currentNode->action.x << ", " << l_currentNode->action.y << ", "
                                           << l_currentNode->action.theta);
        ROS_DEBUG_STREAM("Expanded node velocity: " << l_currentNode->velocity);
        ROS_DEBUG_STREAM("Expanded node world coordinates: " << l_currentNode->worldCoordinates.x << ", "
                                                             << l_currentNode->worldCoordinates.y);

        // Flag signaling if valid footstep found
        bool l_validFootstepFound = false;

        for (float &l_nextVelocity: m_velocities) {
            for (unsigned int i = 0; i < m_numberOfActions; ++i) {
                ROS_DEBUG_STREAM("Action " << m_actions[i].x << ", " << m_actions[i].y << ", " << m_actions[i].theta);
                ROS_INFO_STREAM("Current Velocity: " << l_currentNode->velocity);
                ROS_INFO_STREAM("Next Velocity: " << l_nextVelocity);
                ROS_DEBUG_STREAM("Footstep checked: " << m_validFootstepsFound);
                ROS_INFO_STREAM("Current G: " << l_currentNode->G);
                ROS_INFO_STREAM("Current H: " << l_currentNode->H);

                // Disallow velocities above 0.5 for non-forward actions
                if (m_actions[i].x != 1 && l_nextVelocity > 0.5) {
                    continue;
                }

                World3D l_newWorldCoordinatesCoM{};
                FeetConfiguration l_newFeetConfiguration;

                Node l_tempNode = *l_currentNode;

                // If a new different action than current one is applied,
                // start the robot from an idle configuration.
                if (m_actions[i] != l_currentNode->action && l_currentNode->action != Action{0, 0, 0}) {
                    l_tempNode.velocity = 0.0;
                    l_tempNode.action = Action{0, 0, 0};
                    l_tempNode.feetConfiguration = m_idleFeetConfiguration;
                }

                m_model.predictNextState(l_tempNode.velocity != l_nextVelocity,
                                         l_tempNode.velocity,
                                         l_nextVelocity,
                                         m_actions[i],
                                         p_odomVelocityState,
                                         l_tempNode.worldCoordinates,
                                         l_tempNode.feetConfiguration,
                                         l_newFeetConfiguration,
                                         l_newWorldCoordinatesCoM);

                Vec2D l_newGridCoordinatesCoM{};
                AStar::Search::worldToGrid(l_newWorldCoordinatesCoM, l_newGridCoordinatesCoM);

                if (detectCollision(l_newGridCoordinatesCoM) || findNodeOnList(l_closedSet,
                                                                               m_actions[i],
                                                                               l_nextVelocity,
                                                                               l_newGridCoordinatesCoM,
                                                                               l_newWorldCoordinatesCoM.q)) {
                    continue;
                }

                float l_hindFootCost = 0;
                float l_frontFootCost = 0;
                if (m_validFootstepsFound < FOOTSTEP_HORIZON) {
                    setFeetConfigurationMapFields(l_newWorldCoordinatesCoM, l_newFeetConfiguration);

                    Vec2D l_flGridPose{};
                    Vec2D l_frGridPose{};
                    Vec2D l_rlGridPose{};
                    Vec2D l_rrGridPose{};
                    worldToGrid(l_newFeetConfiguration.flMap, l_flGridPose);
                    worldToGrid(l_newFeetConfiguration.frMap, l_frGridPose);
                    worldToGrid(l_newFeetConfiguration.rlMap, l_rlGridPose);
                    worldToGrid(l_newFeetConfiguration.rrMap, l_rrGridPose);

                    float l_hindFootDistance = 0;
                    float l_frontFootDistance = 0;
                    if (l_currentNode->feetConfiguration.fr_rl_swinging) {
                        if (!m_elevationMapProcessor.validFootstep(l_frGridPose.x, l_frGridPose.y,
                                                                   l_frontFootDistance) ||
                            !m_elevationMapProcessor.validFootstep(l_rlGridPose.x, l_rlGridPose.y,
                                                                   l_hindFootDistance)) {
                            continue;
                        }
                    } else {
                        if (!m_elevationMapProcessor.validFootstep(l_flGridPose.x, l_flGridPose.y,
                                                                   l_frontFootDistance) ||
                            !m_elevationMapProcessor.validFootstep(l_rrGridPose.x, l_rrGridPose.y,
                                                                   l_hindFootDistance)) {
                            continue;
                        }
                    }

                    l_newFeetConfiguration.flMap.z = m_elevationMapProcessor.getCellHeight(l_flGridPose.x,
                                                                                           l_flGridPose.y);
                    l_newFeetConfiguration.frMap.z = m_elevationMapProcessor.getCellHeight(l_frGridPose.x,
                                                                                           l_frGridPose.y);
                    l_newFeetConfiguration.rlMap.z = m_elevationMapProcessor.getCellHeight(l_rlGridPose.x,
                                                                                           l_rlGridPose.y);
                    l_newFeetConfiguration.rrMap.z = m_elevationMapProcessor.getCellHeight(l_rrGridPose.x,
                                                                                           l_rrGridPose.y);

                    l_validFootstepFound = true;
                    l_hindFootCost = (l_hindFootDistance >= MIN_FOOT_DISTANCE) ? 0.0 :
                                     (MIN_FOOT_DISTANCE - l_hindFootDistance);
                    l_frontFootCost = (l_frontFootDistance >= MIN_FOOT_DISTANCE) ? 0.0 :
                                      (MIN_FOOT_DISTANCE - l_frontFootDistance);

                    ROS_INFO_STREAM("Front foot cost/distance: " << l_frontFootCost << "," << l_frontFootDistance);
                    ROS_INFO_STREAM("Hind foot cost/distance: " << l_hindFootCost << "," << l_hindFootDistance);

                    ROS_DEBUG_STREAM(
                            "Heights: " << l_newFeetConfiguration.flMap.z << ", "
                                        << l_newFeetConfiguration.frMap.z << ", "
                                        << l_newFeetConfiguration.rlMap.z << ", "
                                        << l_newFeetConfiguration.rrMap.z);
                    ROS_DEBUG_STREAM(
                            "Coordinates: " << l_flGridPose.x << ", "
                                            << l_flGridPose.y << ", "
                                            << l_rrGridPose.x << ", "
                                            << l_rrGridPose.y);
                }

                l_newWorldCoordinatesCoM.z =
                        p_sourceWorldCoordinates.z + m_elevationMapProcessor.getCellHeight(l_newGridCoordinatesCoM.x,
                                                                                           l_newGridCoordinatesCoM.y);

                Node *successor = findNodeOnList(l_openSet,
                                                 m_actions[i],
                                                 l_nextVelocity,
                                                 l_newGridCoordinatesCoM,
                                                 l_newWorldCoordinatesCoM.q);

                float l_footCost = l_hindFootCost + l_frontFootCost;

                if (successor == nullptr) {
                    successor = new Node(m_actions[i],
                                         l_newGridCoordinatesCoM,
                                         l_newWorldCoordinatesCoM,
                                         l_newFeetConfiguration,
                                         l_currentNode);
                    successor->G = l_footCost;
                    successor->velocity = l_nextVelocity;
                    successor->H = 0 * AStar::Heuristic::euclidean(*successor, Node{Action{0, 0, 0},
                                                                                l_targetGridCoordinates,
                                                                                p_targetWorldCoordinates,
                                                                                l_newFeetConfiguration});
                    l_openSet.push_back(successor);
                } else if (l_footCost < successor->G) {
                    successor->G = l_footCost;
                    successor->action = m_actions[i];
                    successor->parent = l_currentNode;
                    successor->velocity = l_nextVelocity;
                    successor->gridCoordinates = l_newGridCoordinatesCoM;
                    successor->worldCoordinates = l_newWorldCoordinatesCoM;
                    successor->feetConfiguration = l_newFeetConfiguration;
                }
            }
        }

        l_expandedNodes += 1;
        if (l_validFootstepFound) {
            m_validFootstepsFound += 1;
        } else {
            ROS_INFO("Search: No valid footstep found...");
        }

        ros::spinOnce();
    }

    // Add all actions planned except the very first one
    // (i.e. the starting action as this was already executed)
    // which happens to be the root node (i.e. null parent)
    while (l_currentNode != nullptr && l_currentNode->parent != nullptr) {
        ROS_INFO_STREAM("Actions: " << l_currentNode->action.x << ", " << l_currentNode->action.y << ", "
                                    << l_currentNode->action.theta);
        p_path.push_back(*l_currentNode);
        l_currentNode = l_currentNode->parent;

        ros::spinOnce();
    }

    std::reverse(p_path.begin(), p_path.end());

    releaseNodes(l_openSet);
    releaseNodes(l_closedSet);

    ROS_DEBUG_STREAM("Number of expanded nodes: " << l_expandedNodes);
    ROS_DEBUG_STREAM("Path size: " << p_path.size());
}

/**
 * A* Heuristic class' routine that returns the
 * coordinate difference between two points.
 *
 * @param p_sourceWorldCoordinates
 * @param p_targetWorldCoordinates
 * @return points' coordinate difference
 */
World3D AStar::Heuristic::getDistanceDelta(const World3D &p_sourceWorldCoordinates,
                                           const World3D &p_targetWorldCoordinates) {
    return World3D{abs(p_sourceWorldCoordinates.x - p_targetWorldCoordinates.x),
                   abs(p_sourceWorldCoordinates.y - p_targetWorldCoordinates.y)};
}

/**
 * A* Heuristic class routine that computes
 * theta difference between source and target
 *
 * @param p_sourceWorldCoordinates
 * @param p_targetWorldCoordinates
 * @return relative yaw rotation between robot and goal
 */
float AStar::Heuristic::getHeadingDelta(const World3D &p_sourceWorldCoordinates,
                                        const World3D &p_targetWorldCoordinates) {
    // Compute map to target quaternion
    tf2::Quaternion l_targetRotation;
    l_targetRotation = p_targetWorldCoordinates.q;
    l_targetRotation.normalize();

    // Compute relative rotation between
    // target quaternion and CoM quaternion
    tf2::Quaternion l_robotRotation;
    l_robotRotation = p_sourceWorldCoordinates.q;
    l_robotRotation.normalize();
    //ROS_DEBUG_STREAM("Angle of map robot: " << getYawFromQuaternion(l_robotRotation));

    // Return yaw angle in degrees
    return getYawFromQuaternion(l_targetRotation) - getYawFromQuaternion(l_robotRotation);
}

/**
 * A* Heuristic class routine that computes
 * the euclidean distance between two points.
 *
 * @param p_sourceNode
 * @param p_targetNode
 * @return euclidean distance
 */
float AStar::Heuristic::euclidean(const Node &p_sourceNode, const Node &p_targetNode) {
    auto l_angleDelta = getHeadingDelta(p_sourceNode.worldCoordinates, p_targetNode.worldCoordinates);
    auto l_distanceDelta = getDistanceDelta(p_sourceNode.worldCoordinates, p_targetNode.worldCoordinates);

    auto l_angleHeuristic = static_cast<float>(std::abs(l_angleDelta) * 5);
    auto l_distanceHeuristic = static_cast<float>(0.6 * sqrt(pow(l_distanceDelta.x, 2) + pow(l_distanceDelta.y, 2)));

    ROS_DEBUG_STREAM("Angle Delta: " << l_angleDelta);
    ROS_DEBUG_STREAM("Angle heuristic: " << l_angleHeuristic);
    ROS_INFO_STREAM("Distance heuristic: " << l_distanceHeuristic << "\n");

    return l_distanceHeuristic;
}