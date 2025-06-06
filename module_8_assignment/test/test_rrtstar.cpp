#include "gtest/gtest.h"
#include "algo_rrtstar.hpp"
#include <random>
#include <iostream>

void printPath(const std::vector<NodeRRTStar>& path) {
    for (const auto& node : path) {
        std::cout << "(" << node.getX() << ", " << node.getY() << ") -> ";
    }
    std::cout << "Goal" << std::endl;
}

TEST(RRTStarPlannerTest, GenerateRandomNode) {
    RRTStar_Planner planner;
    std::mt19937 gen(0);
    NodeRRTStar randomNode = planner.generateRandomNode(gen);
    EXPECT_GE(randomNode.getX(), 0);
    EXPECT_LT(randomNode.getX(), 410);
    EXPECT_GE(randomNode.getY(), 0);
    EXPECT_LT(randomNode.getY(), 360);
}

TEST(RRTStarPlannerTest, FindNearestNode) {
    RRTStar_Planner planner;
    NodeRRTStar start(0, 0);
    NodeRRTStar node1(2, 2);
    NodeRRTStar node2(5, 5);
    std::vector<NodeRRTStar> nodes = { start, node1, node2 };
    NodeRRTStar randomNode(3, 3);
    NodeRRTStar nearestNode = planner.findNearestNode(nodes, randomNode);
    EXPECT_EQ(nearestNode, node1);
}

TEST(RRTStarPlannerTest, FindNewConfig) {
    RRTStar_Planner planner;
    NodeRRTStar nearestNode(0, 0);
    NodeRRTStar randomNode(2, 2);
    NodeRRTStar newNode = planner.findNewConfig(nearestNode, randomNode);
    float dist = NodeRRTStar::heuristicsEuclid(nearestNode, newNode);
    EXPECT_NEAR(dist, 1.41, 0.01);
    EXPECT_NEAR(newNode.getX(), 1, 0.01);
    EXPECT_NEAR(newNode.getY(), 1, 0.01);
}

TEST(RRTStarPlannerTest, IsObstacle) {
    RRTStar_Planner planner;
    std::vector<int> domain(410 * 360, 0);
    planner.setDomain(domain);
    NodeRRTStar nearestNode(0, 0);
    NodeRRTStar newNode(10, 5);
    EXPECT_FALSE(planner.isObstacle(nearestNode, newNode));

    domain[5 * 410 + 10] = 100; // x:10, y:5 // Equation: y * grid width + x
    planner.setDomain(domain);
    EXPECT_TRUE(planner.isObstacle(nearestNode, newNode));
}

TEST(RRTStarPlannerTest, IsGoalFound) {
    RRTStar_Planner planner;
    NodeRRTStar newNode(5, 5);
    NodeRRTStar goal(5, 5);
    EXPECT_TRUE(planner.isGoalFound(newNode, goal));

    NodeRRTStar otherNode(10, 10);
    EXPECT_FALSE(planner.isGoalFound(otherNode, goal));
}

TEST(RRTStarPlannerTest, FindNearNodes) {
    RRTStar_Planner planner;
    planner.addNode(NodeRRTStar(1, 1));
    planner.addNode(NodeRRTStar(3, 4));
    planner.addNode(NodeRRTStar(7, 7));
    planner.addNode(NodeRRTStar(10, 10));

    NodeRRTStar newNode(2, 2);
    auto nearNodes = planner.findNearNodes(newNode, 5.0);
    EXPECT_GE(nearNodes.size(), 1);
}

TEST(RRTStarPlannerTest, PlanPathBasic) {
    RRTStar_Planner planner;
    std::vector<int> domain(410 * 360, 0);
    planner.setDomain(domain);
    NodeRRTStar start(0, 0);
    NodeRRTStar goal(15, 30);
    std::vector<NodeRRTStar> path = planner.planPath(start, goal);

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_NEAR(path.back().getX(), goal.getX(), 5.0);
    EXPECT_NEAR(path.back().getY(), goal.getY(), 5.0);

    EXPECT_NEAR(NodeRRTStar::heuristicsEuclid(path.back(), goal), 0.0, 5.1);

    std::cout << "Path size: " << path.size() << std::endl;
    printPath(path);
}

TEST(RRTStarPlannerTest, NoPathDueToObstacles) {
    RRTStar_Planner planner;
    std::vector<int> domain(410 * 360, 100); // Entire grid is obstacle
    planner.setDomain(domain);
    NodeRRTStar start(0, 0);
    NodeRRTStar goal(10, 10);
    std::vector<NodeRRTStar> path = planner.planPath(start, goal);

    EXPECT_TRUE(path.empty());
}

TEST(RRTStarPlannerTest, ValidParentConnections) {
    RRTStar_Planner planner;
    std::vector<int> domain(410 * 360, 0);
    planner.setDomain(domain);
    NodeRRTStar start(0, 0);
    NodeRRTStar goal(10, 10);
    std::vector<NodeRRTStar> path = planner.planPath(start, goal);

    ASSERT_FALSE(path.empty());

    for (size_t i = 1; i < path.size(); ++i) {
        EXPECT_EQ(*path[i].getParent(), path[i - 1]);
    }
}

TEST(RRTStarPlannerTest, IndexToCoordinate) {
    RRTStar_Planner planner;

    NodeRRTStar node0 = planner.indexToCoordinate(0); 
    EXPECT_EQ(node0.getX(), 0);
    EXPECT_EQ(node0.getY(), 0);

    NodeRRTStar node1 = planner.indexToCoordinate(410);
    EXPECT_EQ(node1.getX(), 0);
    EXPECT_EQ(node1.getY(), 1);

    int indexMid = 5 * 410 + 10; // (x=10, y=5 => index = x * GRID_WIDTH + y)
    NodeRRTStar nodeMid = planner.indexToCoordinate(indexMid);
    EXPECT_EQ(nodeMid.getX(), 10);
    EXPECT_EQ(nodeMid.getY(), 5);

    NodeRRTStar lastFirstRow = planner.indexToCoordinate(410 - 1); // last cell in first row
    EXPECT_EQ(lastFirstRow.getX(), 409);
    EXPECT_EQ(lastFirstRow.getY(), 0);
}

TEST(RRTStarPlannerTest, Rewire) {
    RRTStar_Planner planner;

    NodeRRTStar newNode(0, 0);
    newNode.setCost(0.0f);

    // initial cost = 10, distance = 5
    NodeRRTStar neighbor1(3, 4);
    neighbor1.setCost(10.0f); // Higher than new cost (if newnode is parent, cost = 5)
    std::shared_ptr<NodeRRTStar> oldParent1 = std::make_shared<NodeRRTStar>(-1, -1);
    neighbor1.setParent(oldParent1);

    // initial cost = 2, distance = 10
    NodeRRTStar neighbor2(6, 8);
    neighbor2.setCost(2.0f); // Lower than new cost (if newnode is parent, cost = 10)
    std::shared_ptr<NodeRRTStar> oldParent2 = std::make_shared<NodeRRTStar>(-2, -2);
    neighbor2.setParent(oldParent2);

    std::vector<NodeRRTStar> nearNodes = {neighbor1, neighbor2};

    planner.rewire(newNode, nearNodes);

    // Rewired: new cost = 0 + 5 = 5 < old cost = 10
    EXPECT_NEAR(nearNodes[0].getCost(), 5.0f, 0.01);
    ASSERT_NE(nearNodes[0].getParent(), nullptr);
    EXPECT_EQ(nearNodes[0].getParent()->getX(), newNode.getX());
    EXPECT_EQ(nearNodes[0].getParent()->getY(), newNode.getY());

    // Not Rewired: old cost = 2 < new cost = 0 + 10 = 10
    EXPECT_NEAR(nearNodes[1].getCost(), 2.0f, 0.01);
    ASSERT_NE(nearNodes[1].getParent(), nullptr);
    EXPECT_EQ(nearNodes[1].getParent()->getX(), oldParent2->getX());
    EXPECT_EQ(nearNodes[1].getParent()->getY(), oldParent2->getY());
}
