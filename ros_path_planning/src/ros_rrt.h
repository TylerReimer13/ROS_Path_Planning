#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>


struct Obstacle {
    std::string type;

    double TopLeft[2]; 
    double TopRight[2]; 
    double BotLeft[2]; 
    double BotRight[2]; 

    double center[2];
    double radius;
};

struct Point {
    double x;
    double y;
    double proximity;
};


struct Node {
    double x;
    double y;
    double proximity;
    int id;
    int parent;
};


class RRT {
    private:
        double START_X;
        double START_Y;

        double GOAL_X;
        double GOAL_Y;
        double THRESHOLD;

        double X_MIN;
        double X_MAX;
        double Y_MIN;
        double Y_MAX;

        double STEP_SIZE;
        int EPSILON;

        int unique_id = 1;
        int MAX_ITER;

    public:
        RRT(double* start, double* goal, double thres, double* x_bounds, double* y_bounds, double step_size, double eps, double iter);
        Point newPoint(std::uniform_real_distribution<double> x_unif, std::uniform_real_distribution<double> y_unif, std::default_random_engine& re);
        static bool sortByDist(const Node &node_1, const Node &node_2) { return node_1.proximity < node_2.proximity;};
        Node closestNode(std::vector<Node>& tree, Point& this_point);
        Node takeStep(Point rand_point, Node nearest_node, double step_size, int& uniq_id);
        void generateObstacles(std::vector<Obstacle>& obstacles);
        bool checkPointCollision(std::vector<Obstacle>& obstacles, Point& possible_point);
        bool checkNodeCollision(std::vector<Obstacle>& obstacles, Node& possible_node);
        bool foundGoal(Point goal_pt, Node node, double THRESHOLD);
        Node getNodefromID(int id, std::vector<Node> Nodes);
        std::vector<Node> getPath(std::vector<Node> Tree, Node last_node);
        std::vector<Node> solve();
	void place_points(ros::Publisher marker_publisher, std::vector<Node> Nodes);
};

