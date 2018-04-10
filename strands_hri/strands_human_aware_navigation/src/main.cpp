#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>

#include "bayes_people_tracker/PeopleTracker.h"

using namespace std;
using namespace bayes_people_tracker;

int threshold;
double scale;
double max_dist, min_dist;

ros::Publisher cmd_pub;
message_filters::Cache<PeopleTracker> *cache;

void callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_DEBUG_STREAM("Received cmd_vel:\n" << *msg);

    // Take last element from cache and check if there is any
    PeopleTracker::ConstPtr pl = cache->getElemBeforeTime(ros::Time::now());
    if(pl != NULL) {
        // Check if detection is too old using the threshold
        if(pl->header.stamp.sec > ros::Time::now().sec-threshold) {
            ROS_DEBUG_STREAM("Found pedestrian localisation msg:\n" << *pl);
            // Calculate new speed using max speed and the location
            double speed = pl->min_distance - min_dist;
            speed = speed > 0.0 ? speed : 0.0;
            speed /= (max_dist - min_dist);
            speed = speed > 1.0 ? 1.0 : speed;
            speed *= scale;
            ROS_DEBUG_STREAM("New speed: " << speed);
            if(speed < msg->linear.x) {
                geometry_msgs::Twist new_msg;
                new_msg.linear.x = speed;
                new_msg.angular.z  = msg->angular.z;
                ROS_DEBUG_STREAM("Publishing: " << new_msg);
                //Publish and return to skip publishing at the end
                cmd_pub.publish(new_msg);
                return;
            }
        } else {
            ROS_DEBUG("No current observation");
        }
    } else {
        ROS_DEBUG("No observation");
    }
    //Publish original message if no person is found
    cmd_pub.publish(*msg);
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "human_aware_cmd_vel");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string pl_topic;
    string cmd_vel_in_topic;
    string cmd_vel_out_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("people_tracker", pl_topic, string("/people_tracker/positions"));
    private_node_handle_.param("cmd_vel_in", cmd_vel_in_topic, string("/human_aware_cmd_vel/input/cmd_vel"));
    private_node_handle_.param("cmd_vel_out", cmd_vel_out_topic, string("/cmd_vel"));
    private_node_handle_.param("threshold", threshold, 3);
    private_node_handle_.param("max_speed", scale, 0.7);
    private_node_handle_.param("max_dist", max_dist, 5.0);
    private_node_handle_.param("min_dist", min_dist, 1.5);

    // Creating a cache for the pedestrian locations because we do not ant to synchronise
    // the /cmd_vel input and the pedestrian localisation. This would prevent the /cmd_vel
    // from being published when there are no humans around.
    message_filters::Subscriber<PeopleTracker> sub(n, pl_topic.c_str(), 1);
    cache = new message_filters::Cache<PeopleTracker>(sub, 10);

    // Create a subscriber.
    ros::Subscriber cmd_sub = n.subscribe(cmd_vel_in_topic.c_str(), 10, &callback);

    // Create a publisher
    cmd_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_out_topic.c_str(), 10);

    ros::spin();
    return 0;
}

