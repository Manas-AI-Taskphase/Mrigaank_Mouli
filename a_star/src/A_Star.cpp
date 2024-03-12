#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

struct Node {
    int x, y;
    double cost;
    double heuristic;
    Node* parent;

    Node(int x_, int y_, double cost_, Node* parent_) :
        x(x_), y(y_), cost(cost_), parent(parent_) {
        // Calculate heuristic dynamically
        // Assuming Euclidean distance as heuristic
        heuristic = 0;
    }
};

struct NodeComparator {
    bool operator()(const Node* a, const Node* b) const {
        return (a->cost + a->heuristic) > (b->cost + b->heuristic);
    }
};

bool operator==(const Node& a, const Node& b) {
    return a.x == b.x && a.y == b.y;
}

std::vector<Node*> AStar(Node* start, Node* goal, const std::vector<std::vector<int>>& map, int width, int height) {
    std::vector<Node*> path;
    std::vector<std::vector<bool>> closed(height, std::vector<bool>(width, false));

    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open;
    start->cost = 0;
    start->heuristic = std::sqrt(std::pow(goal->x - start->x, 2) + std::pow(goal->y - start->y, 2));
    open.push(start);

    while (!open.empty()) {
        Node* current = open.top();
        open.pop();
        
        std::cout<<current->x<<"   "<<current->y<<std::endl;
        std::cout<<goal->x<<"    "<<goal->y<<std::endl;

        
        if (*current == *goal) {
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
             // Reverse the path to get it from start to goal
            std::reverse(path.begin(), path.end());
            break;
        }
	      
	     closed[current->y][current->x] = true;
	      
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = current->x + dx;
                int ny = current->y + dy;

                if (nx >= 0 && nx < width && ny >= 0 && ny < height && map[ny][nx] == 0 && !closed[ny][nx]) {
                    double new_cost = current->cost + std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
                    double heuristic = std::sqrt(std::pow(goal->x - nx, 2) + std::pow(goal->y - ny, 2));

                    Node* neighbor = new Node(nx, ny, new_cost, current);
		    //Check if the neighbor is already in the closed set

                    // Check if the neighbor is already in the open set
       
                    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> openCopy=open;

		     bool InopenSet = false;
                    while (!openCopy.empty()) {
                        Node* openNode = openCopy.top();
                        openCopy.pop();

                        if (*neighbor == *openNode) {
                            
                            InopenSet = true;
                            // Compare costs and update if the new cost is lower
                            if (new_cost < openNode->cost) {
                                openNode->cost = new_cost;
                                openNode->parent = current;
                                open.push(openNode);  // Re-insert the updated node
                            } else {
                                delete neighbor;  // Discard the duplicate neighbor
                            }

                            break;
                        }
                    }
                    if (!InopenSet)
                    {
                       neighbor->heuristic = heuristic;
                    	open.push(neighbor);
                    }

                }
            }
        }
    }

    return path;
}

void cleanupNodes(const std::vector<Node*>& nodes) {
    for (Node* node : nodes) {
        delete node;
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    int width = msg->info.width;
    int height = msg->info.height;

    std::vector<std::vector<int>> map(height, std::vector<int>(width));

    // Populate the map with data from the message
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            map[y][x] = msg->data[index];
        }
    }

    Node* start = new Node(0, 0, 0, nullptr);
    Node* goal = new Node(49, 49, 0, nullptr);

    std::vector<Node*> path = AStar(start, goal, map, width, height);

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = (*it)->x;
        pose.pose.position.y = (*it)->y;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    if (path.empty()) {
        ROS_WARN("No valid path found.");
    } else {
        path_pub.publish(path_msg);
        std::cout<<"path published"<<std::endl;
    }

    cleanupNodes(path);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    ros::spin();

    return 0;
}
