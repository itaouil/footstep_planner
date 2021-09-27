/*
 * AStar.cpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "search/AStar.hpp"

/**
 * Obtain yaw angle (in degrees) from
 * respective quaternion rotation.
 *
 * @param p_quaternion
 * @return yaw angle from quaternion
 */
double AStar::getYawFromQuaternion(const tf2::Quaternion &p_quaternion)
{
    double l_roll, l_pitch, l_yaw;
    tf2::Matrix3x3 l_rotationMatrix(p_quaternion);
    l_rotationMatrix.getRPY(l_roll, l_pitch, l_yaw);
    return ((l_yaw * 180) / M_PI);
}

/**
 * Convert from grid coordinates to world coordinates.
 *
 * @param p_originX
 * @param p_originY
 * @param p_gridCoordinates
 * @param p_worldCoordinates
 */
void AStar::gridToWorld(double p_originX,
                        double p_originY,
                        const Vec2D &p_mapCoordinates,
                        World2D &p_worldCoordinates)
{
    p_worldCoordinates.x = p_originX + p_mapCoordinates.x * HEIGHT_MAP_RESOLUTION;
    p_worldCoordinates.y = p_originY + p_mapCoordinates.y * HEIGHT_MAP_RESOLUTION;
}

/**
 * Convert from world coordinates to grid coordinates.
 *
 * @param p_originX
 * @param p_originY
 * @param p_worldCoordinates
 * @param p_gridCoordinates
 * @return if conversion is successful
 */
bool AStar::worldToGrid(const double p_originX,
                        const double p_originY,
                        const World2D &p_worldCoordinates,
                        Vec2D &p_gridCoordinates)
{
    // Compute top left corner world coordinates of the height map
    double l_topLeftCornerWorldX = p_originX + HEIGHT_MAP_RESOLUTION * ((static_cast<double>(HEIGHT_MAP_GRID_SIZE_X) / 2));
    double l_topLeftCornerWorldY = p_originY + HEIGHT_MAP_RESOLUTION * ((static_cast<double>(HEIGHT_MAP_GRID_SIZE_Y) / 2));
//    ROS_INFO_STREAM("Offset: " << HEIGHT_MAP_RESOLUTION * ((static_cast<double>(HEIGHT_MAP_GRID_SIZE_X) / 2)));
//    ROS_INFO_STREAM("Top left corner: " << l_topLeftCornerX << ", " << l_topLeftCornerY);

    // Compute offset between top left corner and target position
    double l_distanceX = std::abs(l_topLeftCornerWorldX - p_worldCoordinates.x);
    double l_distanceY = std::abs(l_topLeftCornerWorldY - p_worldCoordinates.y);
//    ROS_INFO_STREAM("Distances: " << l_distanceX << ", " << l_distanceY);

    // Compute relative grid position
    p_gridCoordinates.x = (int)(l_distanceX / HEIGHT_MAP_RESOLUTION);
    p_gridCoordinates.y = (int)(l_distanceY / HEIGHT_MAP_RESOLUTION);
//    ROS_INFO_STREAM("Coordinates: " << p_gridCoordinates.x << ", " << p_gridCoordinates.y);

    return true;
}

/**
 * A* search class constructor
 */
AStar::Search::Search(ros::NodeHandle& p_nh): m_model(p_nh)
{
    // Set cost heuristics: manhattan, euclidean, octagonal
    setHeuristic(&Heuristic::euclidean);

    // Set if diagonal movements are allowed
    setDiagonalMovement(SET_DIAGONAL_MOVEMENT);

    // Set 2D height map size
    setGridSize(HEIGHT_MAP_GRID_SIZE_X, HEIGHT_MAP_GRID_SIZE_Y);

    // Available actions
    m_actions = {
            { 1, 0, 0 }, // Forward
            { 0, -1, 0 }, // Right
            { 0, 1, 0 }, // Left
            { 0, 0, -1 }, // Clockwise
            { 0, 0, 1 }, // Counterclockwise
            { 1, 0, -1 }, // Forward + Counterclockwise
            { 1, 0, 1 } // Forward + Counterclockwise
    };

    // Available velocities
    m_velocities = {0.1, 0.2, 0.3, 0.5, 0.7};
}

/**
 * A* search class destructor
 */
AStar::Search::~Search() = default;

/**
 * Set grid map size.
 *
 * @param p_dimensionX
 * @param p_dimensionY
 */
void AStar::Search::setGridSize(const unsigned int p_dimensionX, const unsigned int p_dimensionY)
{
    m_gridSize.x = p_dimensionX;
    m_gridSize.y = p_dimensionY;
}

/**
 * Sets grid origin in static reference frame.
 *
 * @param p_originX
 * @param origin_y
 */
void AStar::Search::setGridOrigin(const double p_originX, const double p_originY)
{
    m_gridOriginX = p_originX;
    m_gridOriginY = p_originY;
}

/**
 * Sets whether the search uses a 5
 * or 7 neighbor expansion for the nodes
 *
 * @param p_enable
 */
void AStar::Search::setDiagonalMovement(bool p_enable)
{
    m_numberOfActions = (p_enable ? 7 : 5);
}

/**
 * Check if expanded coordinate is valid
 * with respect to the grid bounds as well
 * as if the height is acceptable.
 *
 * @param p_gridCoordinates
 * @return if grid cell without bounds
 */
bool AStar::Search::detectCollision(const Vec2D &p_gridCoordinates) const
{
    //TODO: add height check
    if (p_gridCoordinates.x < 0 || p_gridCoordinates.x >= m_gridSize.x ||
        p_gridCoordinates.y < 0 || p_gridCoordinates.y >= m_gridSize.y) {
        ROS_DEBUG("AStar: Collision detected.");
        ROS_DEBUG_STREAM(p_gridCoordinates.x << ", " << m_gridSize.x);
        ROS_DEBUG_STREAM(p_gridCoordinates.y << ", " << m_gridSize.y);
        return true;
    }
    return false;
}

/**
 * Release the node pointers within collection.
 *
 * @param p_nodes
 */
void AStar::Search::releaseNodes(std::vector<Node*> &p_nodes)
{
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
Node *AStar::Search::findNodeOnList(const std::vector<Node*> &p_nodes,
                                    const Action &p_action,
                                    const double p_velocity,
                                    const Vec2D &p_gridCoordinates,
                                    const tf2::Quaternion &p_quaternion)
{
    for (auto &node : p_nodes)
    {
        // Compute heading angle between the nodes
        double l_yaw1 = getYawFromQuaternion(p_quaternion);
        double l_yaw2 = getYawFromQuaternion(node->worldCoordinates.q);
        double l_yawDifference = std::abs(l_yaw2 - l_yaw1);

        // Check if the two nodes have the same state
        // (i.e. same grid coordinates, same yaw, and same velocity)
        if (node->gridCoordinates == p_gridCoordinates &&
            node->action == p_action &&
            node->velocity == p_velocity &&
            l_yawDifference < 0.1)
        {
            ROS_INFO_STREAM("Same node: " << node->gridCoordinates.x << ", "
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
void AStar::Search::setHeuristic(const std::function<unsigned int(Node, Node)>& p_heuristic)
{
    m_heuristic = [p_heuristic](auto && PH1, auto && PH2) {return p_heuristic(std::forward<decltype(PH1)>(PH1),
                                                                              std::forward<decltype(PH2)>(PH2));};
}

/**
 * Reset state feet configuration to
 * the initial source feet configuration
 * (i.e. when robot was idle and not yet
 * moving).
 *
 * @param p_sourceFeetConfiguration
 * @param p_idleFeetConfiguration
 */
void AStar::Search::setIdleFeetConfiguration(const FeetConfiguration &p_sourceFeetConfiguration,
                                             FeetConfiguration &p_idleFeetConfiguration)
{
    p_idleFeetConfiguration = p_sourceFeetConfiguration;
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
                                          const Vec2D &p_targetGridCoordinates)
{
    return (p_nodeGridCoordinates.x - p_targetGridCoordinates.x <= 1) && (p_nodeGridCoordinates.y - p_targetGridCoordinates.y <= 1);
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
std::vector<Node> AStar::Search::findPath(const World2D &p_sourceWorldCoordinates,
                                          const World2D &p_targetWorldCoordinates,
                                          const FeetConfiguration &p_sourceFeetConfiguration)
{
    // Convert source and target world
    // coordinates to grid coordinates
    Vec2D l_sourceGridCoordinates{};
    Vec2D l_targetGridCoordinates{};
    worldToGrid(m_gridOriginX, m_gridOriginY, p_sourceWorldCoordinates, l_sourceGridCoordinates);
    worldToGrid(m_gridOriginX, m_gridOriginY, p_targetWorldCoordinates, l_targetGridCoordinates);

    ROS_DEBUG_STREAM("A* start search called with given source: " << l_sourceGridCoordinates.x << ", " << l_sourceGridCoordinates.y);
    ROS_DEBUG_STREAM("A* start search called with given target: " << l_targetGridCoordinates.x << ", " << l_targetGridCoordinates.y);

    // Create open and closed sets
    // for the search process
    std::vector<Node*> l_openSet, l_closedSet;
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
    while (!l_openSet.empty())
    {
        auto l_iterator = l_openSet.begin();
        l_currentNode = *l_iterator;

        // Find which node in the open
        // set to expand based on cost
        for (auto it = l_openSet.begin(); it != l_openSet.end(); it++)
        {
            auto l_iteratorNode = *it;
            if (l_iteratorNode->getScore() <= l_currentNode->getScore())
            {
                l_currentNode = l_iteratorNode;
                l_iterator = it;
            }
        }

        // Target is the expanded node break condition
        if (l_currentNode->gridCoordinates == l_targetGridCoordinates)
        {
            ROS_INFO("Search: Target goal found");
            break;
        }

//        if (withinTargetTolerance(l_currentNode->gridCoordinates, l_targetGridCoordinates))
//        {
//            ROS_INFO("Search: Target goal found");
//            break;
//        }

        l_closedSet.push_back(l_currentNode);
        l_openSet.erase(l_iterator);

        for (double & l_velocity : m_velocities)
        {
            for (unsigned int i = 0; i < m_numberOfActions; ++i)
            {
                // Start feet configuration
                FeetConfiguration l_currentFeetConfiguration;

                ROS_INFO_STREAM("AStar: Action: " << m_actions[i].x * l_velocity << ", "
                                                       << m_actions[i].y * l_velocity<< ", "
                                                       << m_actions[i].theta * l_velocity);

                // If different action than previous one
                // (and not starting node) start from an
                // idle configuration as the robot has to
                // come to a stop first and then start the
                // new action
                if (m_actions[i] != l_currentNode->action && l_currentNode->action != Action{0, 0, 0})
                {
                    ROS_INFO_STREAM("AStar: Different action being applied. Start with idle config.");
                    setIdleFeetConfiguration(p_sourceFeetConfiguration, l_currentFeetConfiguration);
                }
                else
                {
                    l_currentFeetConfiguration = l_currentNode->feetConfiguration;
                }

                ROS_INFO_STREAM("AStar: Current world coordinates: " << l_currentNode->worldCoordinates.x << ", "
                                                                     << l_currentNode->worldCoordinates.y);

                // Predict new CoM and feet configuration
                World2D l_newWorldCoordinatesCoM{};
                FeetConfiguration l_newFeetConfiguration;
                m_model.predictNewConfiguration(l_velocity,
                                                m_actions[i],
                                                l_currentNode->worldCoordinates,
                                                l_currentFeetConfiguration,
                                                l_newFeetConfiguration,
                                                l_newWorldCoordinatesCoM);

                ROS_INFO_STREAM("AStar: New world coordinates: " << l_newWorldCoordinatesCoM.x << ", "
                                                                      << l_newWorldCoordinatesCoM.y);

                // Convert propagated CoM to grid indexes
                Vec2D l_newGridCoordinatesCoM{};
                AStar::worldToGrid(m_gridOriginX,
                                   m_gridOriginY,
                                   l_newWorldCoordinatesCoM,
                                   l_newGridCoordinatesCoM);

                // Check if obtained CoM was already visited
                // or is outside the grid map boundaries
                if (detectCollision(l_newGridCoordinatesCoM) || findNodeOnList(l_closedSet,
                                                                               m_actions[i],
                                                                               l_velocity,
                                                                               l_newGridCoordinatesCoM,
                                                                               l_newWorldCoordinatesCoM.q))
                {
                    continue;
                }

                ROS_DEBUG_STREAM("Current velocity: " << l_velocity);
                ROS_DEBUG_STREAM("Current action: " << m_actions[i].x << ", " << m_actions[i].y << ", " << m_actions[i].theta);
                ROS_DEBUG_STREAM("New CoM (x,y,theta): " << l_newGridCoordinatesCoM.x << ", " << l_newGridCoordinatesCoM.y << ", " << getYawFromQuaternion(l_newWorldCoordinatesCoM.q));

                unsigned int totalCost = l_currentNode->G + ((i < 4) ? 10 : 14);

                Node *successor = findNodeOnList(l_openSet,
                                                 m_actions[i],
                                                 l_velocity,
                                                 l_newGridCoordinatesCoM,
                                                 l_newWorldCoordinatesCoM.q);
                if (successor == nullptr)
                {
                    successor = new Node(m_actions[i],
                                         l_newGridCoordinatesCoM,
                                         l_newWorldCoordinatesCoM,
                                         l_newFeetConfiguration,
                                         l_currentNode);
                    successor->G = totalCost;
                    successor->velocity = l_velocity;
                    successor->H = m_heuristic(*successor, Node{Action{0, 0, 0},
                                                                l_targetGridCoordinates,
                                                                p_targetWorldCoordinates,
                                                                l_newFeetConfiguration});
                    l_openSet.push_back(successor);
                }
                else if (totalCost < successor->G)
                {
                    successor->G = totalCost;
                    successor->velocity = l_velocity;
                    successor->action = m_actions[i];
                    successor->parent = l_currentNode;
                    successor->worldCoordinates = l_newWorldCoordinatesCoM;
                    successor->feetConfiguration = l_newFeetConfiguration;
                }

                ROS_DEBUG_STREAM("Cost: " << successor->H << "\n");
            }
        }
    }

    // Populate path
    std::vector<Node> path;
    while (l_currentNode != nullptr)
    {
        path.push_back(*l_currentNode);
        l_currentNode = l_currentNode->parent;
    }

    // Reverse path (from source to target)
    std::reverse(path.begin(), path.end());

    // Release resources
    releaseNodes(l_openSet);
    releaseNodes(l_closedSet);

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
                                         const Vec2D &p_targetGridCoordinates)
{
    return{abs(p_sourceGridCoordinates.x - p_targetGridCoordinates.x), abs(p_sourceGridCoordinates.y - p_targetGridCoordinates.y) };
}

/**
 * A* Heuristic class routine that computes
 * theta difference between source and target
 *
 * @param p_sourceWorldCoordinates
 * @param p_targetWorldCoordinates
 * @return relative yaw rotation between robot and goal
 */
double AStar::Heuristic::getHeadingDelta(const World2D &p_sourceWorldCoordinates,
                                         const World2D &p_targetWorldCoordinates)
{
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
unsigned int AStar::Heuristic::manhattan(const Node &p_sourceNode, const Node &p_targetNode)
{
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
unsigned int AStar::Heuristic::euclidean(const Node &p_sourceNode, const Node &p_targetNode)
{
    // Angle and distance differences
    auto l_angleDelta = getHeadingDelta(p_sourceNode.worldCoordinates, p_targetNode.worldCoordinates);
    auto l_distanceDelta = getDistanceDelta(p_sourceNode.gridCoordinates, p_targetNode.gridCoordinates);

    // Euclidean distance
    auto l_angleHeuristic = static_cast<unsigned int>(std::abs(l_angleDelta) * 1000);
    auto l_distanceHeuristic = static_cast<unsigned int>(10 * sqrt(pow(l_distanceDelta.x, 2) + pow(l_distanceDelta.y, 2)));

    ROS_INFO_STREAM("Angle Delta: " << l_angleDelta);
    ROS_INFO_STREAM("Angle heuristic: " << l_angleHeuristic);
    ROS_INFO_STREAM("Distance heuristic: " << l_distanceHeuristic);

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
unsigned int AStar::Heuristic::octagonal(const Node &p_sourceNode, const Node &p_targetNode)
{
    auto delta = getDistanceDelta(p_sourceNode.gridCoordinates, p_targetNode.gridCoordinates);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
