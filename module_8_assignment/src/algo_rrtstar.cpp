#include "algo_rrtstar.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

#define GOAL_RADIUS 5.0

RRTStar_Planner::RRTStar_Planner() {
    DOMAIN.fill(0);
}

void RRTStar_Planner::setDomain(std::vector<int>& domain) {
    std::copy(domain.begin(), domain.end(), DOMAIN.begin());
}

bool RRTStar_Planner::isGoalFound(NodeRRTStar& newNode, NodeRRTStar const& goal) {
    float distance = NodeRRTStar::heuristicsEuclid(newNode, goal);
    return distance <= GOAL_RADIUS;
}

NodeRRTStar RRTStar_Planner::generateRandomNode(std::mt19937& gen) {
    std::uniform_int_distribution<int> distribution(0, GRID_WIDTH * GRID_HEIGHT - 1);
    int random_index = distribution(gen);
    return indexToCoordinate(random_index);
}

NodeRRTStar RRTStar_Planner::findNearestNode(std::vector<NodeRRTStar> const& nodes, NodeRRTStar const& randomNode) {
    NodeRRTStar nearestNode = nodes[0];
    double minDistance = std::numeric_limits<double>::max();

    for (auto const& node : nodes) {
        double distance = NodeRRTStar::heuristicsEuclid(node, randomNode);
        if (distance < minDistance) {
            minDistance = distance;
            nearestNode = node;
        }
    }

    return nearestNode;
}

NodeRRTStar RRTStar_Planner::findNewConfig(NodeRRTStar const& nearestNode, NodeRRTStar const& randomNode) {
    int dx = randomNode.getX() - nearestNode.getX();
    int dy = randomNode.getY() - nearestNode.getY();

    float distance = NodeRRTStar::heuristicsEuclid(nearestNode, randomNode);

    float scaledDx = (distance > 0) ? (dx * STEP_SIZE) / distance : dx;
    float scaledDy = (distance > 0) ? (dy * STEP_SIZE) / distance : dy;

    int x = nearestNode.getX() + static_cast<int>(scaledDx);
    int y = nearestNode.getY() + static_cast<int>(scaledDy);

    return NodeRRTStar(x, y);
}

bool RRTStar_Planner::isObstacle(NodeRRTStar const& nearestNode, NodeRRTStar const& newNode) {
    if (newNode.getX() < 0 || newNode.getX() >= GRID_WIDTH || 
        newNode.getY() < 0 || newNode.getY() >= GRID_HEIGHT) {
        return true;
    }

    if (DOMAIN[newNode.getY() * GRID_WIDTH + newNode.getX()] == 100) {
        return true;
    }

    int dx = newNode.getX() - nearestNode.getX();
    int dy = newNode.getY() - nearestNode.getY();
    int steps = std::max(std::abs(dx), std::abs(dy));

    for (int i = 0; i < steps; i++) {
        float x = nearestNode.getX() + (i * dx) / steps;
        float y = nearestNode.getY() + (i * dy) / steps;

        if (DOMAIN[static_cast<int>(y) * GRID_WIDTH + static_cast<int>(x)] == 100) {
            return true;
        }
    }

    return false;
}

NodeRRTStar RRTStar_Planner::indexToCoordinate(int index) {
    int x = index % GRID_WIDTH;
    int y = index / GRID_WIDTH;
    return NodeRRTStar(x, y);
}

std::vector<NodeRRTStar> RRTStar_Planner::findNearNodes(NodeRRTStar const& new_node, float radius) {
    std::vector<NodeRRTStar> near_nodes;
    for (auto& node : nodes) {
        if (NodeRRTStar::heuristicsEuclid(node, new_node) <= radius) {
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

void RRTStar_Planner::rewire(NodeRRTStar& new_node, std::vector<NodeRRTStar>& near_nodes) {
    for (auto& neighbor : near_nodes) {
        float new_cost = new_node.getCost() + NodeRRTStar::heuristicsEuclid(new_node, neighbor);
        if (new_cost < neighbor.getCost()) {
            neighbor.setParent(std::make_shared<NodeRRTStar>(new_node));
            neighbor.setCost(new_cost);
        }
    }
}

void RRTStar_Planner::addNode(const NodeRRTStar& node) {
    nodes.push_back(node);
}

std::vector<NodeRRTStar> RRTStar_Planner::planPath(NodeRRTStar const& start, NodeRRTStar const& goal) {
    path.clear();
    nodes.clear();
    NodeRRTStar root = start;
    root.setCost(0.0);
    nodes.push_back(root);

    std::mt19937 gen(SEED);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        NodeRRTStar randomNode = generateRandomNode(gen);
        NodeRRTStar nearestNode = findNearestNode(nodes, randomNode);

        if (nearestNode == randomNode) continue;

        NodeRRTStar newNode = findNewConfig(nearestNode, randomNode);

        if (isObstacle(nearestNode, newNode)) continue;

        auto parent = std::make_shared<NodeRRTStar>(nearestNode);
        newNode.setParent(parent);
        newNode.setCost(nearestNode.getCost() + NodeRRTStar::heuristicsEuclid(nearestNode, newNode));

        auto near_nodes = findNearNodes(newNode, NEIGHBOR_RADIUS);

        for (auto& near : near_nodes) {
            float cost = near.getCost() + NodeRRTStar::heuristicsEuclid(near, newNode);
            if (cost < newNode.getCost()) {
                newNode.setParent(std::make_shared<NodeRRTStar>(near));
                newNode.setCost(cost);
            }
        }

        nodes.push_back(newNode);
        rewire(newNode, near_nodes);

        if (isGoalFound(newNode, goal)) {
            path.push_back(newNode);
            auto current = newNode.getParent();
            while (current) {
                path.push_back(*current);
                current = current->getParent();
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
    }

    return path;
}