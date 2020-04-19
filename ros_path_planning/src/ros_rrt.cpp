#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ros_rrt.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

//using namespace std;


RRT::RRT(double* start, double* goal, double thres, double* x_bounds, double* y_bounds, double step_size, double eps, double iter){
    START_X = start[0];
    START_Y = start[1];

    GOAL_X = goal[0];
    GOAL_Y = goal[1];

    THRESHOLD = thres;
    X_MIN = x_bounds[0];
    X_MAX = x_bounds[1];

    Y_MIN = y_bounds[0];
    Y_MAX = y_bounds[1];

    STEP_SIZE = step_size;
    EPSILON = eps;
    MAX_ITER = iter;
};

Point RRT::newPoint(std::uniform_real_distribution<double> x_unif, std::uniform_real_distribution<double> y_unif, std::default_random_engine& re) {
    double x = x_unif(re);
    double y = y_unif(re);
    
    Point new_point;
    new_point.x = x;
    new_point.y = y;

    return new_point;

};

Node RRT::closestNode(std::vector<Node>& tree, Point& this_point) {
    for(int i=0; i<tree.size();i++) {
        double delt_x_sqrd = pow((tree[i].x - this_point.x), 2);
        double delt_y_sqrd = pow((tree[i].y - this_point.y), 2);
        double dist = sqrt(delt_x_sqrd + delt_y_sqrd);
        tree[i].proximity = dist;
    };

    sort(tree.begin(), tree.end(), sortByDist);

    return tree[0];
};

Node RRT::takeStep(Point rand_point, Node nearest_node, double step_size, int& uniq_id) {
    double dir_x = rand_point.x - nearest_node.x;
    double dir_y = rand_point.y - nearest_node.y;
    double dir_x_sqrd = pow(dir_x, 2);
    double dir_y_sqrd = pow(dir_y, 2);
    double magnitude = sqrt(dir_x_sqrd + dir_y_sqrd);

    double unit_x = dir_x/magnitude;
    double unit_y = dir_y/magnitude;

    double step_x = step_size*unit_x;
    double step_y = step_size*unit_y;

    Node new_node;
    new_node.x = nearest_node.x+step_x;
    new_node.y = nearest_node.y+step_y;
    new_node.id = uniq_id;
    new_node.parent = nearest_node.id;

    uniq_id++;

    return new_node;
};

void RRT::generateObstacles(std::vector<Obstacle>& obstacles) {
    Obstacle box;
    box.TopLeft[0] = 1.;
    box.TopLeft[1] = 3.;

    box.TopRight[0] = 2.;
    box.TopRight[1] = 3.;

    box.BotLeft[0] = 1.;
    box.BotLeft[1] = 1.;

    box.BotRight[0] = 2.;
    box.BotRight[1] = 1.;

    obstacles.push_back(box);
};

bool RRT::checkPointCollision(std::vector<Obstacle>& obstacles, Point& possible_point) {

    for(int j=0; j<obstacles.size(); j++) {
        bool in_horz_bounds = possible_point.x >= obstacles[j].TopLeft[0] && possible_point.x <= obstacles[j].TopRight[0];
        bool in_vert_bounds = possible_point.y >= obstacles[j].BotLeft[1] && possible_point.y <= obstacles[j].TopLeft[1];
        if (in_horz_bounds && in_vert_bounds) {
            return true;
        };
    };
    return false;
};

bool RRT::checkNodeCollision(std::vector<Obstacle>& obstacles, Node& possible_node) {

    for(int j=0; j<obstacles.size(); j++) {
        bool in_horz_bounds = possible_node.x >= obstacles[j].TopLeft[0] && possible_node.x <= obstacles[j].TopRight[0];
        bool in_vert_bounds = possible_node.y >= obstacles[j].BotLeft[1] && possible_node.y <= obstacles[j].TopLeft[1];
        if (in_horz_bounds && in_vert_bounds) {
            return true;
        };
    };
    return false;
};

bool RRT::foundGoal(Point goal_pt, Node node, double THRESHOLD) {
    double x_dist_sqrd = pow((goal_pt.x-node.x), 2);
    double y_dist_sqrd = pow((goal_pt.y-node.y), 2);

    double dist = sqrt(x_dist_sqrd+y_dist_sqrd);

    if (dist<THRESHOLD) {
        return true;
    };

    return false;
};

Node RRT::getNodefromID(int id, std::vector<Node> Nodes) {
    for (int i=0; i<Nodes.size(); i++) {
        if (Nodes[i].id == id) {
            return Nodes[i];
        };
    };
};

std::vector<Node> RRT::getPath(std::vector<Node> Tree, Node last_node) {
    std::vector<Node> path;
    path.push_back(last_node);
    Node current_node = last_node;
    Node parent_node;

    int counter = 0;
    while (current_node.id != 0) {
        int parent_id = current_node.parent;
        parent_node = getNodefromID(parent_id, Tree);
        path.push_back(parent_node);
        current_node = parent_node;
    };

    for (int m=0; m<path.size(); m++) {
        //std::cout<<" PATH NODE: "<<path[m].id<<" COORDS: "<<"x: "<<path[m].x<<" ,y: "<<path[m].y<<std::endl;
    };
    return path;
};

std::vector<Node> RRT::solve() {
    std::uniform_real_distribution<double> x_dist(X_MIN, X_MAX);
    std::uniform_real_distribution<double> y_dist(Y_MIN, Y_MAX);

    std::default_random_engine re;

    std::vector<Node> Tree;
    std::vector<Obstacle> Obstacles;
    std::vector<Node> Path;

    generateObstacles(Obstacles);

    Point goal_point;
    goal_point.x = GOAL_X;
    goal_point.y = GOAL_Y;

    Node init_node;
    init_node.x = START_X;
    init_node.y = START_Y;
    init_node.id = 0;
    init_node.parent = 0;

    Tree.push_back(init_node);
    
    int i = 0;

    while (i<MAX_ITER) {
        Point rand_pt = newPoint(x_dist, y_dist, re);

        if (i % EPSILON == 0 && i != 0) {
            rand_pt.x = GOAL_X;
            rand_pt.y = GOAL_Y;
        }

        if (checkPointCollision(Obstacles, rand_pt)) {
            while (checkPointCollision(Obstacles, rand_pt)) {
                rand_pt = newPoint(x_dist, y_dist, re);
            };
        };

        Node closest_node = closestNode(Tree, rand_pt);
        Node new_node = takeStep(rand_pt, closest_node, STEP_SIZE, unique_id);
        //std::cout<<"RAND POINT POS: "<<rand_pt.x<<" "<<rand_pt.y<<" ";
        //std::cout<<"CLOSEST NODE: "<<closest_node.id<<" CLOSEST NODE POS: "<<closest_node.x<<" "<<closest_node.y<<" ";
        //std::cout<<"NEW NODE ID: "<<new_node.id<<" NEW NODE POS: "<<new_node.x<<" "<<new_node.y<<std::endl;

        if (checkNodeCollision(Obstacles, new_node)) {
            i++;
            continue;
            };

        Tree.push_back(new_node);

        if (foundGoal(goal_point, new_node, THRESHOLD)) {
            std::cout<<"*******GOAL FOUND******"<<std::endl;
            Path = getPath(Tree, new_node);
            break;
        };

        i++;
    };
    return Path;
};

void RRT::place_points(ros::Publisher marker_publisher, std::vector<Node> Nodes) {

  visualization_msgs::MarkerArray Points;
  int num_points = Nodes.size();
  Points.markers.resize(num_points);

  for(int i=0; i<num_points; i++) {
     Points.markers[i].header.frame_id = "/my_frame";
     Points.markers[i].header.stamp = ros::Time::now();
     Points.markers[i].ns = "basic_shapes";
     Points.markers[i].id = i;
     Points.markers[i].type = visualization_msgs::Marker::SPHERE;
     Points.markers[i].action = visualization_msgs::Marker::ADD;

     Points.markers[i].pose.position.x = Nodes[i].x;
     Points.markers[i].pose.position.y = Nodes[i].y;
     Points.markers[i].pose.position.z = 0.;
     Points.markers[i].pose.orientation.x = 0.0;
     Points.markers[i].pose.orientation.y = 0.0;
     Points.markers[i].pose.orientation.z = 0.0;
     Points.markers[i].pose.orientation.w = 1.0;

     Points.markers[i].scale.x = .1;
     Points.markers[i].scale.y = .1;
     Points.markers[i].scale.z = .1;

     Points.markers[i].color.r = 0.0f;
     Points.markers[i].color.g = 1.0f;
     Points.markers[i].color.b = 0.0f;
     Points.markers[i].color.a = 1.0;

     //marker.lifetime = ros::Duration();
  }

  marker_publisher.publish(Points);
};


int main ( int argc, char** argv ) {
    double start[2] = {0., 0.};
    double end[2] = {4., 4.};

    double THRESHOLD = .15;

    double x_bounds[2] = {-5., 5.};
    double y_bounds[2] = {-5., 5.};

    double STEP_SIZE = .2;
    int EPSILON = 10;
    int MAX_ITER = 10000;
    std::vector<Node> path;

    RRT my_rrt(start, end, THRESHOLD, x_bounds, y_bounds, STEP_SIZE, EPSILON, MAX_ITER);
    path = my_rrt.solve();

   ros::init(argc, argv, "rrt_path");
   ros::NodeHandle n;
   ros::Rate r(1);
   ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
   while (ros::ok()) {
     my_rrt.place_points(marker_pub, path);
     r.sleep();
   };

   return 0;
}
