#include "algo_rrtstar_node.hpp"
#include <cmath>
#include <memory>
#include <utility>

NodeRRTStar::NodeRRTStar(int x, int y, std::shared_ptr<NodeRRTStar> parent, float cost)
    : x(x), y(y), cost(cost), parent(std::move(parent)) {}

NodeRRTStar::NodeRRTStar(int x, int y) : x(x), y(y), cost(0.0F), parent(nullptr) {}

NodeRRTStar::NodeRRTStar() : x(0), y(0), cost(0.0F), parent(nullptr) {}

void NodeRRTStar::setCost(float cost) { this->cost = cost; }
void NodeRRTStar::setParent(std::shared_ptr<NodeRRTStar> parent) {
  this->parent = std::move(parent);
}

auto NodeRRTStar::getParent() -> std::shared_ptr<NodeRRTStar> { return parent; }
auto NodeRRTStar::getCost() const -> float { return cost; }
auto NodeRRTStar::getX() const -> int { return x; }
auto NodeRRTStar::getY() const -> int { return y; }

bool NodeRRTStar::operator==(const NodeRRTStar& node) const {
    return (x == node.x && y == node.y);
}

auto NodeRRTStar::heuristics(NodeRRTStar const &node_1, NodeRRTStar const &node_2) -> float {
  return std::abs(static_cast<float>(node_1.getX() - node_2.getX())) +
         std::abs(static_cast<float>(node_1.getY() - node_2.getY()));
}

float NodeRRTStar::heuristicsEuclid(NodeRRTStar const &node_1, NodeRRTStar const &node_2) {
  return std::sqrt(std::pow(node_1.getX() - node_2.getX(), 2) +
                   std::pow(node_1.getY() - node_2.getY(), 2));
}
