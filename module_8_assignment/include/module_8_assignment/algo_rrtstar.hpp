#ifndef RRTSTAR_PLANNER_HPP
#define RRTSTAR_PLANNER_HPP

#include "algo_rrtstar_node.hpp"
#include <array>
#include <memory>
#include <random>
#include <vector>

class RRTStar_Planner {
public:
    RRTStar_Planner();
    NodeRRTStar START = NodeRRTStar(0, 0);
    NodeRRTStar GOAL = NodeRRTStar(20, 20);

    void setDomain(std::vector<int>& domain);
    static NodeRRTStar indexToCoordinate(int index);

    NodeRRTStar generateRandomNode(std::mt19937& gen);
    bool isObstacle(NodeRRTStar const& nearest_node, NodeRRTStar const& new_node);

    bool isGoalFound(NodeRRTStar& new_node, NodeRRTStar const& goal);
    NodeRRTStar findNearestNode(std::vector<NodeRRTStar> const& nodes, NodeRRTStar const& random_node);
    NodeRRTStar findNewConfig(NodeRRTStar const& nearest_node, NodeRRTStar const& random_node);

    std::vector<NodeRRTStar> findNearNodes(NodeRRTStar const& new_node, float radius);
    void rewire(NodeRRTStar& new_node, std::vector<NodeRRTStar>& near_nodes);

    std::vector<NodeRRTStar> planPath(NodeRRTStar const& start, NodeRRTStar const& goal);

    void addNode(const NodeRRTStar& node);

private:
    int const MAX_ITERATIONS = 100000;
    int const SEED = 0;

    static constexpr int GRID_WIDTH = 410;
    static constexpr int GRID_HEIGHT = 360;
    static constexpr float STEP_SIZE = 2.0F;
    static constexpr float NEIGHBOR_RADIUS = 10.0F;

    std::array<int, GRID_WIDTH * GRID_HEIGHT> DOMAIN{};
    std::vector<NodeRRTStar> nodes;
    std::vector<NodeRRTStar> path;
};

#endif // RRTSTAR_PLANNER_HPP