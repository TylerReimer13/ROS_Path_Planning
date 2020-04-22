#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "pid_controller.h"
#include "ros_path_planning/waypointMsg.h"
#include "ros_path_planning/waypointArray.h"


#define ABS(N) ((N<0)?(-N):(N))

class SubPub {
   private:
      double ang_KP = .02;
      double ang_KD = .01;
      double ang_limit = .3;

      double vel_KP = .9;
      double vel_KD = 1.25;
      double vel_limit = .35;

      double WAYPOINT_THRESHOLD = .04;

      int waypoint_counter = 0;
      double current_waypoint[2];
      bool destination_reached=false;
      int num_waypoints;

      int COUNT=0;
      bool init_stop_count = false;
      int STOP_COUNT=0;
      bool call_once=true;

   public:
      SubPub();
      void odom_callback (const nav_msgs::Odometry::ConstPtr& msg);
      void imu_callback (const sensor_msgs::Imu::ConstPtr& msg);
      void waypoint_callback (const ros_path_planning::waypointArray& msg);
      geometry_msgs::Twist vel_cmd(double des_heading);
      bool reached_waypoint(double* current_pos, double* current_waypoint);
      double get_dist_to_waypoint(double* current_pos, double* current_waypoint);
      double get_angle_to_waypoint(double delt_y, double delt_x);
      double cross_product(double vector_a[], double vector_b[], double temp[]);
      PID ang_pid{ang_KP, ang_KD, ang_limit};
      PID vel_pid{vel_KP, vel_KD, vel_limit};
      double current_heading;
      double current_position_x;
      double current_position_y;
};

SubPub::SubPub() {

};

void SubPub::odom_callback (const nav_msgs::Odometry::ConstPtr& msg) {
   double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
   double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
   double yaw_rad = std::atan2(siny_cosp, cosy_cosp); //+ 3.141592654;
   double yaw_deg = yaw_rad * 57.296; //CCW

   //ROS_INFO("Position-> x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
   //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   //ROS_INFO("ORIENTATION (RADIANS) -> yaw: [%f]", yaw_rad);
   //ROS_INFO("ORIENTATION (DEGREES) -> yaw: [%f]", yaw_deg);
   current_heading = yaw_deg;
   current_position_x = msg->pose.pose.position.x;
   current_position_y = msg->pose.pose.position.y;
   }; 

void SubPub::imu_callback (const sensor_msgs::Imu::ConstPtr& msg) {
   //ROS_INFO("Angular Velocity-> z: [%f]", msg->angular_velocity.z);
   };

void SubPub::waypoint_callback (const ros_path_planning::waypointArray& msg) {
   current_waypoint[0] = msg.waypoints[waypoint_counter].position[0];
   current_waypoint[1] = msg.waypoints[waypoint_counter].position[1];
   num_waypoints = msg.size;
   };
      
geometry_msgs::Twist SubPub::vel_cmd(double des_heading) {
   COUNT++;
   double position[2] = {current_position_x, current_position_y};
   ROS_INFO("CURRENT WAYPOINT: [%f], [%f] ", current_waypoint[0], current_waypoint[1]);
   double y_dist = current_waypoint[1]-position[1];
   double x_dist = current_waypoint[0]-position[0];
   double desired_heading = get_angle_to_waypoint(y_dist, x_dist);

   geometry_msgs::Twist v_cmd;
   double ang_vel = ang_pid.cmd(desired_heading, current_heading);
   double fwd_vel = .25;
   
   double heading_error = desired_heading-current_heading;   

   if (ABS(heading_error) >= .5 && call_once) {
      ROS_INFO("STOP AND TURN");
      ang_vel = ang_pid.cmd(desired_heading, current_heading);
      fwd_vel = 0.;
      if (ABS(heading_error) <= .6) {
         call_once = false;
      };  
   };

   double Vx = cos(current_heading/57.296);
   double Vy = sin(current_heading/57.296);

   double vector_a[] = { Vx, Vy, 0 };
   double vector_b[] = { x_dist, y_dist, 0 };
   double temp[3];

   double dir = cross_product(vector_a, vector_b, temp);

   if (dir < 0.) {
      ang_vel *= -1.;
      };

   v_cmd.angular.z = ang_vel;
   v_cmd.linear.x = fwd_vel;
   
   if (reached_waypoint(position, current_waypoint) && COUNT>20) {
      ROS_INFO("-----------REACHED WAYPOINT------------");
      v_cmd.angular.z = 0.;
      v_cmd.linear.x = 0.;
      waypoint_counter++;

      if (waypoint_counter > num_waypoints) {
         destination_reached=true;
      }

      init_stop_count = true;
      return v_cmd; 
      };

   if (init_stop_count) {
      ROS_INFO("STARTING STOP COUNT");
      v_cmd.angular.z = 0.;
      v_cmd.linear.x = 0.;
      STOP_COUNT++;
      if (STOP_COUNT>5) {
         init_stop_count = false;
         STOP_COUNT = 0;
         call_once=true;
         };
      };

   if (destination_reached) {
      ROS_INFO("**********DESTINATION REACHED***********");
      v_cmd.angular.z = 0.;
      v_cmd.linear.x = 0.;
   };

   return v_cmd;
   };

bool SubPub::reached_waypoint(double* current_pos, double* current_waypoint) {
   double dist = get_dist_to_waypoint(current_pos, current_waypoint);
   if (dist <= WAYPOINT_THRESHOLD) {
      return true;
   };
   return false;
};

double SubPub::get_dist_to_waypoint(double* current_pos, double* current_waypoint) {
   double x_dist = pow((current_pos[0]-current_waypoint[0]), 2);
   double y_dist = pow((current_pos[1]-current_waypoint[1]), 2);
   double dist = sqrt(x_dist+y_dist); 
   return dist;
};

double SubPub::get_angle_to_waypoint(double delt_y, double delt_x) {
   double angle = atan2(delt_y, delt_x)*57.296;
   return angle;
};

double SubPub::cross_product(double vector_a[], double vector_b[], double temp[]) {
   temp[0] = vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1];
   temp[1] = vector_a[0] * vector_b[2] - vector_a[2] * vector_b[0];
   temp[2] = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0];
   return temp[2];
};
      
    
int main(int argc, char **argv) {

   SubPub my_sub_pub;

   ros::init(argc, argv, "sub_and_pub");
   ros::NodeHandle n;

   ros::Subscriber odom_sub = n.subscribe("odom", 100, &SubPub::odom_callback, &my_sub_pub);
   ros::Subscriber imu_sub = n.subscribe("imu", 100, &SubPub::imu_callback, &my_sub_pub);
   ros::Subscriber waypoint_sub = n.subscribe("waypoints", 100, &SubPub::waypoint_callback, &my_sub_pub);
   ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

   ros::Rate loop_rate(10);

   while (ros::ok()) {
      geometry_msgs::Twist v_cmd;
      v_cmd = my_sub_pub.vel_cmd(0.);
      velocity_pub.publish(v_cmd);
      //ROS_INFO("CURRENT HEADING: [%f]", curr_hdg);
      ros::spinOnce();
      loop_rate.sleep();
   };

   ros::spin();

   return 0;
}
