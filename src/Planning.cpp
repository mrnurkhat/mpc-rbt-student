#include "Planning.hpp"

using namespace std::chrono_literals;

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/plan_path", 
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the map service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for map service...");
        }

        requestMap();
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::requestMap() {
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->map.info.width == 0 || response->map.info.height == 0) {
            RCLCPP_ERROR(this->get_logger(), "Received empty map (0x0). Map server not ready. Retrying in 1s...");
            std::this_thread::sleep_for(1s); 
            requestMap();
            return;
        }

        this->map_ = response->map;
        dilateMap();
        RCLCPP_INFO(this->get_logger(), "Map received! Size: %dx%d, resolution: %.2f", 
                    map_.info.width, map_.info.height, map_.info.resolution);

    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Checking map before planning. Current data size: %zu", map_.data.size());
    if (map_.data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Map is empty! Cannot plan path yet.");
        return;
    }        

    RCLCPP_INFO(this->get_logger(), "Service called: Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                request->start.pose.position.x, request->start.pose.position.y,
                request->goal.pose.position.x, request->goal.pose.position.y);

    aStar(request->start, request->goal);
    response->plan = path_; 
    smoothPath();

    path_pub_->publish(path_); 
}

void PlanningNode::dilateMap() {
    if (map_.data.empty()) return;

    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    
    int dilation_radius = 6; 

    int width = map_.info.width;
    int height = map_.info.height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (map_.data[x + y * width] == 100) {
                
                // Раздуваем это препятствие
                for (int dy = -dilation_radius; dy <= dilation_radius; dy++) {
                    for (int dx = -dilation_radius; dx <= dilation_radius; dx++) {
                        
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            dilatedMap.data[nx + ny * width] = 100;
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;
    RCLCPP_INFO(this->get_logger(), "Map dilation finished with radius %d", dilation_radius);
}


int getTop(std::vector<std::shared_ptr<Cell>>& list) {
    if (list.empty()) return -1;

    int best_idx = 0;
    for (int i = 0; i < list.size(); i++) {
        if (list[i]->f < list[best_idx]->f) 
            best_idx = i;
    }

    return best_idx;
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    std::vector<bool> closed_list(map_.info.height * map_.info.width, false);
    std::vector<std::shared_ptr<Cell>> open_list;
    std::vector<std::shared_ptr<Cell>> all_cells(map_.info.height * map_.info.width, nullptr);

    int start_x = worldToGridX(start.pose.position.x);
    int start_y = worldToGridY(start.pose.position.y);

    int goal_x = worldToGridX(goal.pose.position.x);
    int goal_y = worldToGridY(goal.pose.position.y);

    RCLCPP_ERROR(get_logger(), "A* started\n");

    auto start_cell = std::make_shared<Cell>(start_x, start_y);
    start_cell->g = 0.0;
    start_cell->h = std::hypot(goal_x - start_x, goal_y - start_y);
    start_cell->f = start_cell->g + start_cell->h;
    start_cell->parent = nullptr;

    open_list.push_back(start_cell);

    all_cells[start_x + start_y * map_.info.width] = start_cell;

    std::shared_ptr<Cell> curr_cell = nullptr;
    bool path_found = false;

    while (!open_list.empty() && rclcpp::ok()) {
        int best_idx = getTop(open_list);
        curr_cell = open_list[best_idx];
        open_list.erase(open_list.begin() + best_idx);

        int curr_idx = curr_cell->x + curr_cell->y * map_.info.width;
        closed_list[curr_idx] = true;

        if (curr_cell->x == goal_x && curr_cell->y == goal_y) {
            path_found = true;
            break;
        }

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i == 0 && j == 0) continue;

                int n_x = curr_cell->x + i;
                int n_y = curr_cell->y + j;

                if (n_x < 0 || n_x >= (int)map_.info.width || n_y < 0 || n_y >= (int)map_.info.height)
                    continue;

                int n_idx = n_x + n_y * map_.info.width;

                if (map_.data[n_idx] == 100 || closed_list[n_idx])
                    continue;

                float dist_to_neighboor = ((i == 0) || (j == 0)) ? 1.0 : 1.414;

                float g_new = curr_cell->g + dist_to_neighboor;
                
                auto n_cell = all_cells[n_idx];

                if (n_cell == nullptr) {
                    n_cell = std::make_shared<Cell>(n_x, n_y);
                    n_cell->g = g_new;
                    n_cell->h = std::hypot(goal_x - n_x, goal_y - n_y);
                    n_cell->f = n_cell->g + n_cell->h;
                    n_cell->parent = curr_cell;

                    all_cells[n_idx] = n_cell;
                    open_list.push_back(n_cell);
                } else if (g_new < n_cell->g) {
                    n_cell->g = g_new;
                    n_cell->f = n_cell->g + n_cell->h;
                    n_cell->parent = curr_cell;
                }
            }
        }
    }

    if (!path_found) {
        RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    } else {
        path_.poses.clear();
        path_.header.stamp = this->now();
        path_.header.frame_id = "map";

        while (curr_cell != nullptr) {
            geometry_msgs::msg::PoseStamped pose;

            pose.header = path_.header;

            pose.pose.position.x = gridToWorldX(curr_cell->x);
            pose.pose.position.y = gridToWorldY(curr_cell->y);
            pose.pose.orientation.w = 1.0;

            path_.poses.push_back(pose);

            curr_cell = curr_cell->parent;
        }

        std::reverse(path_.poses.begin(), path_.poses.end());
    }
}

void PlanningNode::smoothPath() {
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    float alpha = 0.7;
    for (int i = 1; i < newPath.size() - 1; i++) {
        newPath[i].pose.position.x= newPath[i].pose.position.x * alpha + (1 - alpha) * newPath[i-1].pose.position.x; 
        newPath[i].pose.position.y = newPath[i].pose.position.y * alpha + (1 - alpha) * newPath[i-1].pose.position.y;
    }

    for (int i = newPath.size() - 2; i > 0; i--) {
        newPath[i].pose.position.x= newPath[i].pose.position.x * alpha + (1 - alpha) * newPath[i+1].pose.position.x; 
        newPath[i].pose.position.y = newPath[i].pose.position.y * alpha + (1 - alpha) * newPath[i+1].pose.position.y;
    }

    path_.poses = newPath;
}

Cell::Cell(int c, int r) : x(c), y(r) {
    f = 0.0; g = 0.0; h = 0.0;
    parent = nullptr;
}

int PlanningNode::worldToGridX(double world_x) {
    int grid_x = static_cast<int>((world_x - map_.info.origin.position.x) / map_.info.resolution);
    return grid_x;
}

int PlanningNode::worldToGridY(double world_y) {
    int grid_y = static_cast<int>((world_y - map_.info.origin.position.y) / map_.info.resolution);
    return grid_y;
}

double PlanningNode::gridToWorldX(int grid_x) {
    double world_x = ((grid_x * map_.info.resolution) + map_.info.origin.position.x + map_.info.resolution * 0.5); 
    return world_x;
}

double PlanningNode::gridToWorldY(int grid_y) {
    double world_y = ((grid_y * map_.info.resolution) + map_.info.origin.position.y + map_.info.resolution * 0.5); 
    return world_y;
}