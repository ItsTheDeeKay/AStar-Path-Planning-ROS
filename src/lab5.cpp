#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

Eigen::MatrixXd grid;
Eigen::MatrixXd heuristic;
geometry_msgs::Point goal_node;
geometry_msgs::Point target;
double x, y, theta;

// This is the Callback function for Husky's odometry...
void husky_Odom(const nav_msgs::Odometry::ConstPtr& data) {
    x = data->pose.pose.position.x;
    y = data->pose.pose.position.y;
    
    tf::Quaternion q(
        data->pose.pose.orientation.x,
        data->pose.pose.orientation.y,
        data->pose.pose.orientation.z,
        data->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta);
}

// Function to calculate i, j from x, y
pair<int, int> calculate_ij(double x, double y) {
    int j = static_cast<int>((10 - y) / 0.5);
    int i = static_cast<int>((10 + x) / 0.5);
    return make_pair(i, j);
}

void heu_Cal(const geometry_msgs::Point& goal_node) {
    int row = grid.size();
    int col = grid[0].size();
    
    heuristic.resize(row, vector<int>(col, 0));
    
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            int row_diff = abs(i - goal_node.x);
            int col_diff = abs(j - goal_node.y);
            heuristic[i][j] = abs(row_diff - col_diff) + min(row_diff, col_diff) * 2;
        }
    }
}

void absolute_Position(const vector<pair<int, int>>& node) {
    vector<geometry_msgs::Point> AP_POINTS;
    for (auto& [i, j] : node) {
        geometry_msgs::Point point;
        point.x = -10 + (0.5 * j);
        point.y = 10 - (0.5 * i);
        AP_POINTS.push_back(point);
    }
    
    // TODO: Printing nodes and absolute positions (depends on how you want to display them)

    drive_Husky(AP_POINTS);
}

void drive_Husky(const vector<geometry_msgs::Point>& address) {
    // For simplicity, we'll assume the hardcoded coordinates are a global vector named hardcoded_Coordinates
    // The type should be vector<geometry_msgs::Point>

    vector<geometry_msgs::Point> full_Execution = address;
    size_t point_index = 0;
    geometry_msgs::Point goal;

    ros::Rate rate(4);
    while (ros::ok()) {
        if (point_index < full_Execution.size()) {
            goal = full_Execution[point_index];
        } else {
            // Reached the goal
            ROS_INFO("GOAL REACHED..!! The controller will now get terminated.");
            break;
        }

        double d2X = goal.x - x;
        double d2Y = goal.y - y;
        double angle_to_goal = atan2(d2Y, d2X);
        double d2Goal = sqrt(d2X * d2X + d2Y * d2Y);

        geometry_msgs::Twist husky_Speed;
        if (d2Goal >= 0.2) {
            if (abs(angle_to_goal - theta) > 0.2) {
                husky_Speed.linear.x = 0.0;
                husky_Speed.angular.z = 0.9;
            } else {
                ROS_INFO("Proximity: %f", d2Goal);
                husky_Speed.linear.x = 0.8;
                husky_Speed.angular.z = 0.0;
            }
            pub.publish(husky_Speed);
        } else {
            point_index++;
        }
        rate.sleep();
    }
}

void Astar(const pair<int, int>& start, const pair<int, int>& target) {
    geometry_msgs::Point goal;
    goal.x = target.second;
    goal.y = target.first;

    geometry_msgs::Point init;
    init.x = start.second;
    init.y = start.first;

    heu_Cal(goal);
    vector<pair<int, int>> Astar(pair<int, int> start, pair<int, int> goal, vector<vector<int>>& grid, vector<vector<float>>& heuristic) {
    priority_queue<Node*, vector<Node*>, Compare> openSet;
    unordered_set<int> closedSet; // Hashed as y * gridWidth + x

    vector<pair<int, int>> neighbours = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

    openSet.push(new Node(start.first, start.second, 0, heuristic[start.first][start.second]));

    Node* finalNode = nullptr;

    while (!openSet.empty()) {
        Node* currentNode = openSet.top();
        openSet.pop();

        if (currentNode->x == goal.first && currentNode->y == goal.second) {
            finalNode = currentNode;
            break;
        }

        for (const auto& neighbour : neighbours) {
            int newX = currentNode->x + neighbour.first;
            int newY = currentNode->y + neighbour.second;

            if (newX < 0 || newY < 0 || newX >= grid.size() || newY >= grid[0].size()) continue;
            if (grid[newX][newY] == 1) continue;

            int hash = newY * grid[0].size() + newX;
            if (closedSet.find(hash) == closedSet.end()) {
                float newG = currentNode->g + 1;
                openSet.push(new Node(newX, newY, newG, heuristic[newX][newY], currentNode));
                closedSet.insert(hash);
            }
        }
    }

    if (!finalNode) {
        cout << "No path found." << endl;
        return {};
    }

    vector<pair<int, int>> path;
    while (finalNode) {
        path.push_back({finalNode->x, finalNode->y});
        finalNode = finalNode->parent;
    }
    reverse(path.begin(), path.end());

    // TODO: Clean up allocated Node objects
    // for(auto node : openSet) {
    //     delete node;
    // }

    return path;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Astar");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/odometry/filtered", 1000, husky_Odom);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

    // Setup the grid (like the map variable)
    grid = Eigen::MatrixXd::Zero(42, 42);
    grid = [[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            [1,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
            [1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
            [1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
            [1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
            [1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1],
            [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]]

    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
