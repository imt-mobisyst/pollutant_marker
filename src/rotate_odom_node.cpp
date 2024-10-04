#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher odom_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Create a new Odometry message to publish
    nav_msgs::Odometry transformed_odom = *msg;

    // NED to ENU conversion
    // https://github.com/mavlink/mavros/issues/49
    double ned_x = msg->pose.pose.position.x;
    double ned_y = msg->pose.pose.position.y;
    double ned_z = msg->pose.pose.position.z;

    // ENU coordinates
    transformed_odom.pose.pose.position.x = -ned_x;  // Swap X and Y for ENU
    transformed_odom.pose.pose.position.y = -ned_y;
    transformed_odom.pose.pose.position.z = -ned_z; // Invert Z for ENU

    // Extract the original orientation quaternion (NED)
    tf2::Quaternion ned_orientation(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    // Convert the NED quaternion to ENU by swapping X, Y and inverting Z
    tf2::Quaternion enu_orientation;
    enu_orientation.setX(ned_orientation.getX());
    enu_orientation.setY(ned_orientation.getY());
    enu_orientation.setZ(-ned_orientation.getZ());
    enu_orientation.setW(ned_orientation.getW());

    // Now, we rotate by 90 degrees around the Z-axis (ENU upward)
    tf2::Quaternion z_rot;
    z_rot.setRPY(0, M_PI, 0);  //M_PI 180-degree rotation around Z-axis

    // Apply the 90-degree rotation to the ENU orientation
    tf2::Quaternion final_orientation = z_rot * enu_orientation;
    final_orientation.normalize();  // Normalize to ensure valid quaternion

    // Set the transformed orientation to the new Odometry message
    transformed_odom.pose.pose.orientation.x = final_orientation.getX();
    transformed_odom.pose.pose.orientation.y = final_orientation.getY();
    transformed_odom.pose.pose.orientation.z = final_orientation.getZ();
    transformed_odom.pose.pose.orientation.w = final_orientation.getW();

    // Publish the transformed odometry message
    odom_pub.publish(transformed_odom);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_transformer");
    ros::NodeHandle nh;

    // Log node start
    ROS_INFO("Starting odom node with correct transformation...");
    // Subscribe to mavros/local_position/odom
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, odomCallback);

    // Publisher for transformed /odom
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    ros::spin();
    return 0;
}
