/*
 * AStar.cpp
 *
 *  Created on: Aug 23, 2021
 *      Author: Ilyass Taouil
 *   Institute: University of Bonn, AIS lab
 */

#include "search/AStar.hpp"

/**
 * Equality operator for the Vec2i struct.
 *
 * @param coordinates_
 * @return if r.h.s and l.h.s objects are equal
 */
bool AStar::Vec2i::operator == (const AStar::Vec2i &coordinates_) const
{
    return (x == coordinates_.x && y == coordinates_.y);
}

/**
 * Add operator for the Vec2i struct.
 *
 * @param left_
 * @param right_
 * @return sum of two Vec2i objects
 */
AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

/**
 * A* Node struct constructor.
 *
 * @param coord_
 * @param parent_
 */
AStar::Node::Node(AStar::Vec2i coord_, AStar::Node *parent_)
{
    parent = parent_;
    coordinates = coord_;
    G = H = 0;
}

/**
 * A* Node struct routine that returns
 * the total cost of the node.
 *
 * @return total node cost H(s) + G(s)
 */
unsigned int AStar::Node::getScore() const
{
    return G + H;
}

/**
 * A* search class constructor
 */
AStar::Search::Search()
{
    // Set cost heuristics: manhattan, euclidean, octagonal
    setHeuristic(&Heuristic::euclidean);

    // Set if diagonal movements are allowed
    setDiagonalMovement(SET_DIAGONAL_MOVEMENT);

    // Set 2D height map size
    setWorldSize({HEIGHT_MAP_MAX_SIZE_X, HEIGHT_MAP_MAX_SIZE_Y});

    // Available directions
    direction = {
            { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
            { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

/**
 * A* search class destructor
 */
AStar::Search::~Search() = default;

/**
 * Set grid size.
 *
 * @param worldSize_
 */
void AStar::Search::setWorldSize(AStar::Vec2i worldSize_)
{
    worldSize = worldSize_;
}

/**
 * Sets whether the search uses a 4
 * or 8 neighbor expansion for the nodes
 *
 * @param enable_
 */
void AStar::Search::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

/**
 * Check if expanded coordinate is valid
 * with respect to the grid bounds as well
 * as if the height is acceptable.
 *
 * @param coordinates_
 * @return if grid cell without bounds
 */
bool AStar::Search::detectCollision(AStar::Vec2i coordinates_)
{
    //TODO: add height check
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y) {
        return true;
    }
    return false;
}

/**
 * Release the node pointers within collection.
 *
 * @param nodes_
 */
void AStar::Search::releaseNodes(std::vector<Node*> &nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

/**
 * Find if given node is in a vector (open/closed set).
 *
 * @param nodes_
 * @param coordinates_
 * @return the requested node or a nullptr
 */
AStar::Node *AStar::Search::findNodeOnList(std::vector<Node*> &nodes_, AStar::Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

/**
  * Sets heuristic to be used for the H cost.
  *
  * @param heuristic_
  */
void AStar::Search::setHeuristic(const std::function<unsigned int(Vec2i, Vec2i)>& heuristic_)
{
    heuristic = [heuristic_](auto && PH1, auto && PH2) { return heuristic_(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); };
}

/**
 * Find path from source to target
 * in a given height map.
 *
 * @param source_
 * @param target_
 * @return sequence of 2D points (grid cells indexes)
 */
std::vector<AStar::Vec2i> AStar::Search::findPath(AStar::Vec2i source_, AStar::Vec2i target_)
{
    Node *current = nullptr;
    std::vector<Node*> openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (unsigned int i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            unsigned int totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    std::vector<Vec2i> path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

/**
 * A* Heuristic class' routine that returns the
 * coordinate difference between two points.
 *
 * @param source_
 * @param target_
 * @return points' coordinate difference
 */
AStar::Vec2i AStar::Heuristic::getDelta(AStar::Vec2i source_, AStar::Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

/**
 * A* Heuristic class routine that computes
 * the manhattan distance between two points.
 *
 * @param source_
 * @param target_
 * @return manhattan distance
 */
unsigned int AStar::Heuristic::manhattan(AStar::Vec2i source_, AStar::Vec2i target_)
{
    auto delta = getDelta(source_, target_);
    return static_cast<unsigned int>(10 * (delta.x + delta.y));
}

/**
 * A* Heuristic class routine that computes
 * the euclidean distance between two points.
 *
 * @param source_
 * @param target_
 * @return euclidean distance
 */
unsigned int AStar::Heuristic::euclidean(AStar::Vec2i source_, AStar::Vec2i target_)
{
    auto delta = getDelta(source_, target_);
    return static_cast<unsigned int>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

/**
 * A* Heuristic class routine that computes
 * the octagonal distance between two points.
 *
 * @param source_
 * @param target_
 * @return octagonal distance
 */
unsigned int AStar::Heuristic::octagonal(AStar::Vec2i source_, AStar::Vec2i target_)
{
    auto delta = getDelta(source_, target_);
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
