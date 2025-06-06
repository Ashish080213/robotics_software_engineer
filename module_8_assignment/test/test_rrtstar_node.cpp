#include <gtest/gtest.h>
#include "algo_rrtstar_node.hpp"

TEST(NodeRRTStarTest, DefaultConstructor) {
    NodeRRTStar node(5, 5);
    EXPECT_EQ(node.getX(), 5);
    EXPECT_EQ(node.getY(), 5);
    EXPECT_EQ(node.getCost(), 0.0f);
    EXPECT_EQ(node.getParent(), nullptr);
}

TEST(NodeRRTStarTest, ParameterizedConstructor) {
    auto parent = std::make_shared<NodeRRTStar>(0, 0);
    NodeRRTStar node(5, 10, parent, 5.0f);
    EXPECT_EQ(node.getX(), 5);
    EXPECT_EQ(node.getY(), 10);
    EXPECT_EQ(node.getCost(), 5.0f);
    EXPECT_EQ(node.getParent(), parent);
}

TEST(NodeRRTStarTest, SetParent) {
    auto parent = std::make_shared<NodeRRTStar>(2, 2);
    NodeRRTStar node(1, 1);
    node.setParent(parent);
    EXPECT_EQ(node.getParent(), parent);
}

TEST(NodeRRTStarTest, SetCost) {
    NodeRRTStar node(1, 1);
    node.setCost(2.0f);
    EXPECT_EQ(node.getCost(), 2.0f);
}

TEST(NodeRRTStarTest, EqualityOperator) {
    NodeRRTStar node1(2, 2);
    NodeRRTStar node2(2, 2);
    NodeRRTStar node3(3, 3);
    EXPECT_TRUE(node1 == node2);
    EXPECT_FALSE(node1 == node3);
}

TEST(NodeRRTStarTest, Heuristics) {
    NodeRRTStar node1(0, 0);
    NodeRRTStar node2(6, 8);
    EXPECT_EQ(NodeRRTStar::heuristics(node1, node2), 14.0f);
    EXPECT_NEAR(NodeRRTStar::heuristicsEuclid(node1, node2), 10.0, 1e-5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
