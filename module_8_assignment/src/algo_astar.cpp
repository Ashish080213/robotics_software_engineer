#include "algo_astar.h"

NodeAstar::NodeAstar(int x, int y, std::shared_ptr<NodeAstar> parent)
    : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(parent) {}

void NodeAstar::set_gcost(float cost) {
    g_cost = cost;
    f_cost = g_cost + h_cost;
}

void NodeAstar::set_hcost(float cost) {
    h_cost = cost;
    f_cost = g_cost + h_cost;
}

bool compare_node::operator()(const std::shared_ptr<NodeAstar>& a, const std::shared_ptr<NodeAstar>& b) const {
    return a->f_cost > b->f_cost;
}

float heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

std::pair<int, int> indexToCoordinates(int index, int width) {
  if (index < 0 || index >= width * width) {
    std::cout << "Index out of bounds" << std::endl;
    return {-1, -1};
  }

  int y = index / width;
  int x = index % width;

  return {x, y};
}

std::vector<geometry_msgs::msg::PoseStamped>
astar_search(const nav_msgs::msg::OccupancyGrid& grid, const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal) {

  std::cout << "Started Astar Algorithm" << std::endl;
  std::vector<geometry_msgs::msg::PoseStamped> path;

  int GRID_HEIGHT = grid.info.height;
  int GRID_WIDTH = grid.info.width;
  const auto &grid_data = grid.data;

  int free_cells = 0, occupied_cells = 0;

  for (auto cell : grid_data) {
    if (cell == 0) {
      free_cells++;
    } else if (cell == 100) {
      occupied_cells++;
    }
  }

  std::cout << "Free cells: " << free_cells
            << ", Occupied cells: " << occupied_cells << std::endl;
  std::cout << "Map Width: " << GRID_WIDTH << ", Map Height: " << GRID_HEIGHT << std::endl;

  // Convert start and goal positions to grid coordinates
  int start_x = (start.position.x - grid.info.origin.position.x) / grid.info.resolution;
  int start_y = (start.position.y - grid.info.origin.position.y) / grid.info.resolution;
  int goal_x = (goal.position.x - grid.info.origin.position.x) / grid.info.resolution;
  int goal_y = (goal.position.y - grid.info.origin.position.y) / grid.info.resolution;

  std::cout << "Converted Start point: (" << start_x << ", " << start_y << ")" << std::endl;
  std::cout << "Converted Goal point: (" << goal_x << ", " << goal_y << ")" << std::endl;

  // Ensure start and goal are within bounds
  if (start_x < 0 || start_x >= GRID_WIDTH || start_y < 0 || start_y >= GRID_HEIGHT ||
      goal_x < 0 || goal_x >= GRID_WIDTH || goal_y < 0 || goal_y >= GRID_HEIGHT) {
    std::cout << "Start or Goal point out of bounds." << std::endl;
    return path;  // Return empty path
  }

  std::priority_queue<std::shared_ptr<NodeAstar>, std::vector<std::shared_ptr<NodeAstar>>, compare_node> open_list;
  std::vector<std::vector<float>> cost_so_far(GRID_HEIGHT, std::vector<float>(GRID_WIDTH, std::numeric_limits<float>::max()));
  std::vector<std::vector<bool>> closed_list(GRID_HEIGHT, std::vector<bool>(GRID_WIDTH, false));

  auto start_node = std::make_shared<NodeAstar>(start_x, start_y);
  start_node->set_gcost(0);
  start_node->set_hcost(heuristic(start_node->x, start_node->y, goal_x, goal_y));

  open_list.push(start_node);
  cost_so_far[start_x][start_y] = 0;

  std::vector<std::pair<int, int>> directions = {
      {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};

  while (!open_list.empty()) {
    auto current_node = open_list.top();
    open_list.pop();

    std::cout << "Checking node at (" << current_node->x << ", "
              << current_node->y << ") against goal (" << goal_x << ", "
              << goal_y << ")" << std::endl;

    if (current_node->x == goal_x && current_node->y == goal_y) {
      std::vector<geometry_msgs::msg::PoseStamped> path_reversed;

      auto path_node = current_node;
      int node_count = 0;

      while (path_node != nullptr) {
        node_count++;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = grid.header.frame_id;
        pose.header.stamp = grid.header.stamp;
        pose.pose.position.x = path_node->x * grid.info.resolution + grid.info.origin.position.x;
        pose.pose.position.y = path_node->y * grid.info.resolution + grid.info.origin.position.y;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1.0;

        path_reversed.push_back(pose);
        path_node = path_node->parent;
      }

      std::cout << "Found goal path. Total number of nodes in the path: "
                << node_count << std::endl;

      std::vector<geometry_msgs::msg::PoseStamped> path;
      for (auto it = path_reversed.rbegin(); it != path_reversed.rend(); ++it) {
        path.push_back(*it);
      }
      return path;
    }

    closed_list[current_node->x][current_node->y] = true;

    for (auto dir : directions) {
      int new_x = current_node->x + dir.first;
      int new_y = current_node->y + dir.second;

      if (new_x >= 0 && new_x < GRID_WIDTH && new_y >= 0 && new_y < GRID_HEIGHT) {
        int new_index = new_y * GRID_WIDTH + new_x;
        if (grid_data[new_index] == 100) {
          continue;
        }

        float new_cost = current_node->g_cost + ((dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2));

        if (!closed_list[new_x][new_y]) {
          auto neighbor = std::make_shared<NodeAstar>(new_x, new_y, current_node);

          if (new_cost < cost_so_far[new_x][new_y]) {
            neighbor->set_gcost(new_cost);
            neighbor->set_hcost(heuristic(neighbor->x, neighbor->y, goal_x, goal_y));
            open_list.push(neighbor);
            cost_so_far[new_x][new_y] = new_cost;
          }
        }
      }
    }
  }

  std::cout << "No path found." << std::endl;
  return path;
}
