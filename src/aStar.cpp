/*
 * AStar.cpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "aStar.hpp"

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
 * Convert from grid coordinates to world coordinates.
 *
 * @param p_gridCoordinates
 * @param p_worldCoordinates
 */
void AStar::Search::gridToWorld(const Vec2D &p_mapCoordinates,
                                World3D &p_worldCoordinates) {
    p_worldCoordinates.x = m_elevationMapGridOriginX + p_mapCoordinates.x * m_elevationMapGridResolution;
    p_worldCoordinates.y = m_elevationMapGridOriginX + p_mapCoordinates.y * m_elevationMapGridResolution;
    p_worldCoordinates.z = m_elevationMapProcessor.getCellHeight(p_mapCoordinates.x, p_mapCoordinates.y);
}

/**
 * Convert from world coordinates to grid coordinates.
 *
 * @param p_worldCoordinates
 * @param p_gridCoordinates
 * @return if conversion is successful
 */
bool AStar::Search::worldToGrid(const World3D &p_worldCoordinates,
                                Vec2D &p_gridCoordinates) const {
    // Compute top left corner world coordinates of the height map
    double l_topLeftCornerWorldX = m_elevationMapGridOriginX +
                                   m_elevationMapGridResolution * ((static_cast<double>(m_elevationMapGridSizeX) / 2));
    double l_topLeftCornerWorldY = m_elevationMapGridOriginX +
                                   m_elevationMapGridResolution * ((static_cast<double>(m_elevationMapGridSizeY) / 2));

    ROS_DEBUG_STREAM("Offset: " << m_elevationMapGridResolution * ((static_cast<double>(m_elevationMapGridSizeX) / 2)));
    ROS_DEBUG_STREAM("Top left corner: " << l_topLeftCornerWorldX << ", " << l_topLeftCornerWorldY);

    // Compute offset between top left corner and target position
    double l_distanceX = std::abs(l_topLeftCornerWorldX - p_worldCoordinates.x);
    double l_distanceY = std::abs(l_topLeftCornerWorldY - p_worldCoordinates.y);

    ROS_DEBUG_STREAM("Distances: " << l_distanceX << ", " << l_distanceY);

    // Compute relative grid position
    p_gridCoordinates.x = (int) (l_distanceX / m_elevationMapGridResolution);
    p_gridCoordinates.y = (int) (l_distanceY / m_elevationMapGridResolution);

    ROS_DEBUG_STREAM("Coordinates: " << p_gridCoordinates.x << ", " << p_gridCoordinates.y);

    return true;
}

/**
 * A* search class constructor
 */
AStar::Search::Search(ros::NodeHandle &p_nh) :
        m_model(p_nh),
        m_footstepsChecked(0),
        m_listener(m_buffer),
        m_elevationMapProcessor(p_nh) {
    // Set cost heuristics: manhattan, euclidean, octagonal
    setHeuristic(&Heuristic::euclidean);

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
    m_velocities = {0.1, 0.2, 0.3, 0.4, 0.5};
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
        ROS_DEBUG("AStar: Collision detected.");
        ROS_DEBUG_STREAM(p_gridCoordinates.x << ", " << m_elevationMapGridSizeX);
        ROS_DEBUG_STREAM(p_gridCoordinates.y << ", " << m_elevationMapGridSizeY);
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
 * @return the requested node or a nullptr
 */
Node *AStar::Search::findNodeOnList(const std::vector<Node *> &p_nodes,
                                    const Action &p_action,
                                    const double p_velocity,
                                    const Vec2D &p_gridCoordinates,
                                    const tf2::Quaternion &p_quaternion) {
    for (auto &node: p_nodes) {
        // Compute heading angle between the nodes
        double l_yaw1 = getYawFromQuaternion(p_quaternion);
        double l_yaw2 = getYawFromQuaternion(node->worldCoordinates.q);
        double l_yawDifference = std::abs(l_yaw2 - l_yaw1);

        // Check if the two nodes have the same state
        // (i.e. same grid coordinates, same yaw, and same velocity)
        if (node->gridCoordinates == p_gridCoordinates &&
            node->action == p_action &&
            node->velocity == p_velocity &&
            l_yawDifference < 0.1) {
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
  * Sets heuristic to be used for the H cost.
  *
  * @param p_heuristic
  */
void AStar::Search::setHeuristic(const std::function<unsigned int(Node, Node)> &p_heuristic) {
    m_heuristic = [p_heuristic](auto &&PH1, auto &&PH2) {
        return p_heuristic(std::forward<decltype(PH1)>(PH1),
                           std::forward<decltype(PH2)>(PH2));
    };
}

/**
 * Check if current node coordinates
 * are within the target tolerance
 * distance.
 *
 * @param p_nodeGridCoordinates
 * @param p_targetGridCoordinates
 * @return if coordinates within distance tolerance
 */
bool AStar::Search::withinTargetTolerance(const Vec2D &p_nodeGridCoordinates,
                                          const Vec2D &p_targetGridCoordinates) {
    return (p_nodeGridCoordinates.x - p_targetGridCoordinates.x <= 2) &&
           (p_nodeGridCoordinates.y - p_targetGridCoordinates.y <= 2);
}

/**
 * Transform feet configuration
 * from CoM frame to map frame.
 *
 * @param p_newCoMWorldCoordinates
 * @param p_newFeetConfigurationCoM
 * @param p_newFeetConfigurationMap
 */
void AStar::Search::transformCoMFeetConfigurationToMap(const World3D &p_newCoMWorldCoordinates,
                                                       const FeetConfiguration &p_newFeetConfigurationCoM,
                                                       FeetConfiguration &p_newFeetConfigurationMap) {
    // Fill in new map feet configuration (only x and y position)
    p_newFeetConfigurationMap.flMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfigurationCoM.flCoM.x;
    p_newFeetConfigurationMap.flMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfigurationCoM.flCoM.y;
    p_newFeetConfigurationMap.frMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfigurationCoM.frCoM.x;
    p_newFeetConfigurationMap.frMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfigurationCoM.frCoM.y;
    p_newFeetConfigurationMap.rlMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfigurationCoM.rlCoM.x;
    p_newFeetConfigurationMap.rlMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfigurationCoM.rlCoM.y;
    p_newFeetConfigurationMap.rrMap.x = p_newCoMWorldCoordinates.x + p_newFeetConfigurationCoM.rrCoM.x;
    p_newFeetConfigurationMap.rrMap.y = p_newCoMWorldCoordinates.y + p_newFeetConfigurationCoM.rrCoM.y;
}

/**
 * Find path from source to target
 * in a given height map.
 *
 * @param p_sourceWorldCoordinates
 * @param p_targetWorldCoordinates
 * @param p_sourceFeetConfiguration
 * @return sequence of 2D points (world coordinates)
 */
std::vector<Node> AStar::Search::findPath(const World3D &p_sourceWorldCoordinates,
                                          const World3D &p_targetWorldCoordinates,
                                          const FeetConfiguration &p_sourceFeetConfiguration) {
    // Update elevation map parameters
    m_elevationMapProcessor.getElevationMapParameters(m_elevationMapGridOriginX,
                                                      m_elevationMapGridOriginY,
                                                      m_elevationMapGridResolution,
                                                      m_elevationMapGridSizeX,
                                                      m_elevationMapGridSizeY);

    // Number of expanded nodes so far
    unsigned int l_expandedNodes = 0;

    // Convert source and target world coordinates to grid coordinates
    Vec2D l_sourceGridCoordinates{};
    Vec2D l_targetGridCoordinates{};
    worldToGrid(p_sourceWorldCoordinates, l_sourceGridCoordinates);
    worldToGrid(p_targetWorldCoordinates, l_targetGridCoordinates);

    ROS_DEBUG_STREAM("A* start search called with given source: " << l_sourceGridCoordinates.x << ", "
                                                                  << l_sourceGridCoordinates.y);
    ROS_DEBUG_STREAM("A* start search called with given target: " << l_targetGridCoordinates.x << ", "
                                                                  << l_targetGridCoordinates.y);

    // Create open and closed sets for the search process
    std::vector<Node *> l_openSet, l_closedSet;
    l_openSet.reserve(100);
    l_closedSet.reserve(100);

    // Currently expanded node
    Node *l_currentNode = nullptr;

    // Push to initial open set the source node
    l_openSet.push_back(new Node(Action{0, 0, 0},
                                 l_sourceGridCoordinates,
                                 p_sourceWorldCoordinates,
                                 p_sourceFeetConfiguration));

    // Search process
    while (!l_openSet.empty()) {
        l_expandedNodes += 1;

        auto l_iterator = l_openSet.begin();
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

        // Stop search if target node has been expanded
//        if (l_currentNode->gridCoordinates == l_targetGridCoordinates) {
//            ROS_INFO("Search: Target goal found");
//            break;
//        }

        if (withinTargetTolerance(l_currentNode->gridCoordinates, l_targetGridCoordinates))
        {
            ROS_INFO("Search: Target goal found");
            break;
        }

        // Push to closed set the currently expanded
        // node and delete its iterator from the open set
        l_closedSet.push_back(l_currentNode);
        l_openSet.erase(l_iterator);

        for (double &l_nextVelocity: m_velocities) {
            for (unsigned int i = 0; i < m_numberOfActions; ++i) {
                // Next robot state
                FeetConfiguration l_currentFeetConfiguration;

                // If different action than previous one
                // (and not starting node) start from an
                // idle configuration as the robot has to
                // come to a stop first and then start the
                // new action
                if (m_actions[i] != l_currentNode->action && l_currentNode->action != Action{0, 0, 0}) {
                    ROS_INFO_STREAM("AStar: Different action being applied. Start with idle config.");
                    l_currentFeetConfiguration = p_sourceFeetConfiguration;
                } else {
                    l_currentFeetConfiguration = l_currentNode->feetConfiguration;
                }

                // Predict new robot state (CoM and Feet)
                World3D l_newCoMWorldCoordinates{};
                FeetConfiguration l_newFeetConfiguration;
                m_model.predictNextState(l_currentNode->velocity != l_nextVelocity,
                                         l_currentNode->velocity,
                                         l_nextVelocity,
                                         m_actions[i],
                                         l_currentNode->worldCoordinates,
                                         l_currentFeetConfiguration,
                                         l_newFeetConfiguration,
                                         l_newCoMWorldCoordinates);

                // Convert new world CoM to grid indexes
                Vec2D l_newGridCoordinatesCoM{};
                AStar::Search::worldToGrid(l_newCoMWorldCoordinates, l_newGridCoordinatesCoM);

                // Check if new CoM was already visited or outside the boundaries
                if (detectCollision(l_newGridCoordinatesCoM) || findNodeOnList(l_closedSet,
                                                                               m_actions[i],
                                                                               l_nextVelocity,
                                                                               l_newGridCoordinatesCoM,
                                                                               l_newCoMWorldCoordinates.q)) {
                    continue;
                }

                // Perform foothold validity for pre-defined horizon
                if (m_footstepsChecked <= FOOTSTEP_HORIZON) {
                    // Transform feet configuration from CoM to Map frame
                    FeetConfiguration l_newFeetConfigurationMap;
                    transformCoMFeetConfigurationToMap(l_newCoMWorldCoordinates,
                                                       l_newFeetConfiguration,
                                                       l_newFeetConfigurationMap);

                    ROS_DEBUG_STREAM("FL map position: " << l_newFeetConfigurationMap.flMap.x << ", "
                                                         << l_newFeetConfigurationMap.flMap.y);

                    ROS_DEBUG_STREAM("FR map position: " << l_newFeetConfigurationMap.frMap.x << ", "
                                                         << l_newFeetConfigurationMap.frMap.y);

                    ROS_DEBUG_STREAM("RL map position: " << l_newFeetConfigurationMap.rlMap.x << ", "
                                                         << l_newFeetConfigurationMap.rlMap.y);

                    ROS_DEBUG_STREAM("RR map position: " << l_newFeetConfigurationMap.rrMap.x << ", "
                                                         << l_newFeetConfigurationMap.rrMap.y << "\n");

                    // Convert footholds to grid indices
                    Vec2D l_flGridPose{};
                    Vec2D l_frGridPose{};
                    Vec2D l_rlGridPose{};
                    Vec2D l_rrGridPose{};
                    worldToGrid(l_newFeetConfigurationMap.flMap, l_flGridPose);
                    worldToGrid(l_newFeetConfigurationMap.frMap, l_frGridPose);
                    worldToGrid(l_newFeetConfigurationMap.rlMap, l_rlGridPose);
                    worldToGrid(l_newFeetConfigurationMap.rrMap, l_rrGridPose);

                    // Check if foot grid indexes are in valid terrain
                    if (!m_elevationMapProcessor.checkFootholdValidity(l_flGridPose.x, l_flGridPose.y) ||
                        !m_elevationMapProcessor.checkFootholdValidity(l_frGridPose.x, l_frGridPose.y) ||
                        !m_elevationMapProcessor.checkFootholdValidity(l_rlGridPose.x, l_rlGridPose.y) ||
                        !m_elevationMapProcessor.checkFootholdValidity(l_rrGridPose.x, l_rrGridPose.y)) {
                        ROS_INFO("Invalid Footstep.");
                        continue;
                    }

                    // Add map feet poses to feet configuration
                    l_newFeetConfiguration.flMap = l_newFeetConfigurationMap.flMap;
                    l_newFeetConfiguration.frMap = l_newFeetConfigurationMap.frMap;
                    l_newFeetConfiguration.rlMap = l_newFeetConfigurationMap.rlMap;
                    l_newFeetConfiguration.rrMap = l_newFeetConfigurationMap.rrMap;

                    // Set new feet configuration world height
                    l_newFeetConfiguration.flMap.z = m_elevationMapProcessor.getCellHeight(l_flGridPose.x,
                                                                                           l_flGridPose.y);
                    l_newFeetConfiguration.frMap.z = m_elevationMapProcessor.getCellHeight(l_frGridPose.x,
                                                                                           l_frGridPose.y);
                    l_newFeetConfiguration.rlMap.z = m_elevationMapProcessor.getCellHeight(l_rlGridPose.x,
                                                                                           l_rlGridPose.y);
                    l_newFeetConfiguration.rrMap.z = m_elevationMapProcessor.getCellHeight(l_rrGridPose.x,
                                                                                           l_rrGridPose.y);
                }

                // Set new CoM world height
                l_newCoMWorldCoordinates.z =
                        p_sourceWorldCoordinates.z + m_elevationMapProcessor.getCellHeight(l_newGridCoordinatesCoM.x,
                                                                                           l_newGridCoordinatesCoM.y);

                unsigned int totalCost = l_currentNode->G + ((i < 4) ? 10 : 14);

                Node *successor = findNodeOnList(l_openSet,
                                                 m_actions[i],
                                                 l_nextVelocity,
                                                 l_newGridCoordinatesCoM,
                                                 l_newCoMWorldCoordinates.q);
                if (successor == nullptr) {
                    successor = new Node(m_actions[i],
                                         l_newGridCoordinatesCoM,
                                         l_newCoMWorldCoordinates,
                                         l_newFeetConfiguration,
                                         l_currentNode);
                    successor->G = totalCost;
                    successor->velocity = l_nextVelocity;
                    successor->H = m_heuristic(*successor, Node{Action{0, 0, 0},
                                                                l_targetGridCoordinates,
                                                                p_targetWorldCoordinates,
                                                                l_newFeetConfiguration});
                    l_openSet.push_back(successor);
                } else if (totalCost < successor->G) {
                    successor->G = totalCost;
                    successor->velocity = l_nextVelocity;
                    successor->action = m_actions[i];
                    successor->parent = l_currentNode;
                    successor->worldCoordinates = l_newCoMWorldCoordinates;
                    successor->feetConfiguration = l_newFeetConfiguration;
                }

                // Increase footsteps validation counter
                m_footstepsChecked += 1;
            }
        }
    }

    // Populate path
    std::vector<Node> path;
    while (l_currentNode != nullptr) {
        path.push_back(*l_currentNode);
        l_currentNode = l_currentNode->parent;
    }

    // Reverse path (from source to target)
    std::reverse(path.begin(), path.end());

    // Release resources
    releaseNodes(l_openSet);
    releaseNodes(l_closedSet);

    ROS_DEBUG_STREAM("Number of expanded nodes: " << l_expandedNodes);

    return path;
}

/**
 * A* Heuristic class' routine that returns the
 * coordinate difference between two points.
 *
 * @param p_sourceGridCoordinates
 * @param p_targetGridCoordinates
 * @return points' coordinate difference
 */
Vec2D AStar::Heuristic::getDistanceDelta(const Vec2D &p_sourceGridCoordinates,
                                         const Vec2D &p_targetGridCoordinates) {
    return {abs(p_sourceGridCoordinates.x - p_targetGridCoordinates.x),
            abs(p_sourceGridCoordinates.y - p_targetGridCoordinates.y)};
}

/**
 * A* Heuristic class routine that computes
 * theta difference between source and target
 *
 * @param p_sourceWorldCoordinates
 * @param p_targetWorldCoordinates
 * @return relative yaw rotation between robot and goal
 */
double AStar::Heuristic::getHeadingDelta(const World3D &p_sourceWorldCoordinates,
                                         const World3D &p_targetWorldCoordinates) {
    // Compute map to target quaternion
    tf2::Quaternion l_mapToTargetQuaternion;
    l_mapToTargetQuaternion.setRPY(0, 0, std::atan2(p_targetWorldCoordinates.y, p_targetWorldCoordinates.x));

    // Compute relative rotation between
    // target quaternion and CoM quaternion
    tf2::Quaternion l_CoMToTargetQuaternion;
    l_CoMToTargetQuaternion = l_mapToTargetQuaternion * p_sourceWorldCoordinates.q.inverse();
    l_CoMToTargetQuaternion.normalize();

    // Return yaw angle in degrees
    return getYawFromQuaternion(l_CoMToTargetQuaternion);
}

/**
 * A* Heuristic class routine that computes
 * the manhattan distance between two points.
 *
 * @param p_sourceNode
 * @param p_targetNode
 * @return manhattan distance
 */
unsigned int AStar::Heuristic::manhattan(const Node &p_sourceNode, const Node &p_targetNode) {
    auto l_distanceDelta = getDistanceDelta(p_sourceNode.gridCoordinates, p_targetNode.gridCoordinates);
    return static_cast<unsigned int>(10 * (l_distanceDelta.x + l_distanceDelta.y));
}

/**
 * A* Heuristic class routine that computes
 * the euclidean distance between two points.
 *
 * @param p_sourceNode
 * @param p_targetNode
 * @return euclidean distance
 */
unsigned int AStar::Heuristic::euclidean(const Node &p_sourceNode, const Node &p_targetNode) {
    // Angle and distance differences
    auto l_angleDelta = getHeadingDelta(p_sourceNode.worldCoordinates, p_targetNode.worldCoordinates);
    auto l_distanceDelta = getDistanceDelta(p_sourceNode.gridCoordinates, p_targetNode.gridCoordinates);

    // Euclidean distance
    auto l_angleHeuristic = static_cast<unsigned int>(std::abs(l_angleDelta) * 1000);
    auto l_distanceHeuristic = static_cast<unsigned int>(10 *
                                                         sqrt(pow(l_distanceDelta.x, 2) + pow(l_distanceDelta.y, 2)));

    ROS_DEBUG_STREAM("Angle Delta: " << l_angleDelta);
    ROS_DEBUG_STREAM("Angle heuristic: " << l_angleHeuristic);
    ROS_DEBUG_STREAM("Distance heuristic: " << l_distanceHeuristic);

    return l_angleHeuristic + l_distanceHeuristic;
}

/**
 * A* Heuristic class routine that computes
 * the octagonal distance between two points.
 *
 * @param p_sourceNode
 * @param p_targetNode
 * @return octagonal distance
 */
unsigned int AStar::Heuristic::octagonal(const Node &p_sourceNode, const Node &p_targetNode) {
    auto delta = getDistanceDelta(p_sourceNode.gridCoordinates, p_targetNode.gridCoordinates);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
