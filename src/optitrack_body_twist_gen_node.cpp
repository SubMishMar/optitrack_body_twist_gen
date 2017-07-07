#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include <optitrack_body_twist_gen/or_pose_estimator_state.h>
using Eigen::MatrixXd;
using namespace std;
class convertor
{
        public:

                convertor();

                void chatterCallback(const optitrack_body_twist_gen::or_pose_estimator_state::ConstPtr& msg);
                

        private:
                ros::NodeHandle nh_;
                ros::Publisher pub;
                ros::Subscriber sub;
                

                double ex, ey, ez, ew;
                double x, y, z;
                geometry_msgs::Pose previousPose;
                nav_msgs::Odometry body_odom;
                ros::Time previousTimestamp;

};


convertor::convertor()
{
        // Initialize all position variables and orientation variables
        //mocap variables
        cout << " In the constructor " << endl;
        x = y = z = ex = ey = ez = ew = 0;
        sub = nh_.subscribe("/optitrack/bodies/brunelleschi", 1000, &convertor::chatterCallback, this);
        pub = nh_.advertise<nav_msgs::Odometry>("/brunelleschi/odom",100);
        previousTimestamp = ros::Time(0);
}

void convertor::chatterCallback(const optitrack_body_twist_gen::or_pose_estimator_state::ConstPtr& msg)
{

    //If empty messages are not taken care then the node crashes
    if((msg -> pos).empty())
      return;

    ex = msg -> pos[0].qx;
    ey = msg -> pos[0].qy;
    ez = msg -> pos[0].qz;
    ew = msg -> pos[0].qw;

    x = msg -> pos[0].x;
    y = msg -> pos[0].y;
    z = msg -> pos[0].z;
   
    
    body_odom.header.stamp = ros::Time::now();
    body_odom.pose.pose.position.x = x;
    body_odom.pose.pose.position.y = y;
    body_odom.pose.pose.position.z = z;

    body_odom.pose.pose.orientation.x = ex;
    body_odom.pose.pose.orientation.y = ey;
    body_odom.pose.pose.orientation.z = ez;
    body_odom.pose.pose.orientation.w = ew;

    if(previousTimestamp.toSec() == 0)
    {
     body_odom.twist.twist.linear.x = 0;
     body_odom.twist.twist.linear.y = 0;
     body_odom.twist.twist.linear.z = 0;
     body_odom.twist.twist.angular.x = 0;
     body_odom.twist.twist.angular.y = 0;
     body_odom.twist.twist.angular.z = 0;
     cout << "The beginning of time...." << endl;
    }
    else
    {
     ros::Duration diff = body_odom.header.stamp - previousTimestamp;
     double seconds = diff.toSec();
     tf::Quaternion diffAngle = tf::Quaternion(previousPose.orientation.x,previousPose.orientation.y,previousPose.orientation.z,previousPose.orientation.w) * tf::Quaternion(body_odom.pose.pose.orientation.x,body_odom.pose.pose.orientation.y,body_odom.pose.pose.orientation.z,body_odom.pose.pose.orientation.w).inverse();

     body_odom.twist.twist.linear.x = (previousPose.position.x-body_odom.pose.pose.position.x)/seconds;
     body_odom.twist.twist.linear.y = (previousPose.position.y-body_odom.pose.pose.position.y)/seconds;
     body_odom.twist.twist.linear.z = (previousPose.position.z-body_odom.pose.pose.position.z)/seconds;

     tf::Matrix3x3 diffMat(diffAngle);
     diffMat.getRPY(body_odom.twist.twist.angular.x,body_odom.twist.twist.angular.y,body_odom.twist.twist.angular.z);
     body_odom.twist.twist.angular.x /=seconds;
     body_odom.twist.twist.angular.y /=seconds;
     body_odom.twist.twist.angular.z /=seconds;
    }
    pub.publish(body_odom);
    previousTimestamp = body_odom.header.stamp;
    previousPose.position = body_odom.pose.pose.position;
    previousPose.orientation = body_odom.pose.pose.orientation;
   
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "optitrack_body_twist_gen_node");

        convertor rs;

        ros::Rate loop_rate(50);
        while (ros::ok())
        {

            ros::spinOnce();
            loop_rate.sleep();

        }

        return 0;
}
