#ifndef RRTSTAR_NODE_HPP
#define RRTSTAR_NODE_HPP

#include <memory>
#include <vector>

class NodeRRTStar {
public:
    NodeRRTStar(int x, int y, std::shared_ptr<NodeRRTStar> parent, float cost);
    NodeRRTStar(int x, int y);
    NodeRRTStar();

    void setParent(std::shared_ptr<NodeRRTStar> parent);
    void setCost(float cost);

    auto getParent() -> std::shared_ptr<NodeRRTStar>;
    auto getCost() const -> float;
    auto getX() const -> int;
    auto getY() const -> int;
    bool operator==(const NodeRRTStar& node) const;

    static auto heuristics(const NodeRRTStar& node_1, const NodeRRTStar& node_2) -> float;
    static float heuristicsEuclid(const NodeRRTStar& node_1, const NodeRRTStar& node_2);

private:
    int x, y;
    float cost = 0.0F;
    std::shared_ptr<NodeRRTStar> parent = nullptr;
};

#endif // RRTSTAR_NODE_HPP
