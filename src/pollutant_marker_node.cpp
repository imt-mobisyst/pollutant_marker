#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

// First pollutant pose
double pol1_dist_x = -14.2;
double pol1_dist_y = 7.2;

// Second pollutant pose
double pol2_dist_x = 2.4;
double pol2_dist_y = -35.0;

// Third pollutant pose
double pol3_dist_x = 28.7;
double pol3_dist_y = -21.7;

// Threshold for Euclidean distance
double threshold_dist_1 = 15.0;
double threshold_dist_2 = 4.5;
double threshold_dist_3 = 2.5;

ros::Publisher marker_array_pub;
visualization_msgs::MarkerArray marker_array; // Store marker array
int marker_id = 0;

// Function to compute Euclidean distance
double computeDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void publishStaticTransform() {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    
    // Set translation, which can be adjusted based on the setup
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    // Set rotation to identity quaternion
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    // Publish the transform
    static_broadcaster.sendTransform(transformStamped);
}

// Callback function for the odom topic of Mavros
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // Compute Euclidean distances to pollutants
    double distance_1 = computeDistance(current_x, current_y, pol1_dist_x, pol1_dist_y);
    double distance_2 = computeDistance(current_x, current_y, pol2_dist_x, pol2_dist_y);
    double distance_3 = computeDistance(current_x, current_y, pol3_dist_x, pol3_dist_y);

    // Create a new marker for the current odometry position
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_namespace";
    marker.id = marker_id++; // Unique ID for each marker
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set marker orientation and consistent scale
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;  // Uniform scaling for visibility
    marker.scale.y = 0.5;
    marker.scale.z = 0.05;

    // Set marker position based on current odometry
    marker.pose.position.x = current_x;
    marker.pose.position.y = current_y;
    marker.pose.position.z = 0.0;

    // Determine marker color based on proximity to pollutants
    if ((distance_1 > threshold_dist_1) && (distance_2 > threshold_dist_1) && (distance_3 > threshold_dist_1)) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0; // Green for safe
        marker.color.a = 1.0;
        ROS_INFO("Robot is in the safe zone...");
    } else if (((distance_1 > threshold_dist_2) && (distance_1 < threshold_dist_1)) || 
               ((distance_2 > threshold_dist_2) && (distance_2 < threshold_dist_1)) || 
               ((distance_3 > threshold_dist_2) && (distance_3 < threshold_dist_1))) {
        marker.color.r = 1.0;
        marker.color.g = 0.65;
        marker.color.b = 0.0; // Orange for caution
        marker.color.a = 1.0;
        ROS_WARN("Robot is close to pollutant zone. Publishing orange alert...");
    } else {
        marker.color.r = 1.0; // Red for danger
        marker.color.g = 0.0;
        marker.color.a = 1.0;
        ROS_ERROR("Robot is in pollutant zone. Publishing danger...");
    }

    // Append the new marker to the marker array
    marker_array.markers.push_back(marker);

    // Publish the updated marker array
    marker_array_pub.publish(marker_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pollutant_marker_node");
    ros::NodeHandle nh;

    // Log node start
    ROS_INFO("Starting pollutant indicator node...");

    // Publish static transform
    publishStaticTransform();

    // Subscriber to the /odom topic
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Publisher for the marker array
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::spin();
    return 0;
}
