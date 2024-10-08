#include <cstdio>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;
template <typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
    T ans;
    std::string default_value = "";
    n->declare_parameter<std::string>(name, default_value);
    if (n->get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_ERROR_STREAM(n->get_logger(), "Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;
}

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF)
        {
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};
int idx = 1;
vector<Data> benchmark;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
nav_msgs::msg::Path path;

int init = 0;
Quaterniond baseRgt;
Vector3d baseTgt;

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    //ROS_INFO("odom callback!");
    if ((odom_msg->header.stamp.sec+odom_msg->header.stamp.nanosec*(1e-9)) > benchmark.back().t)
      return;
  
    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= (odom_msg->header.stamp.sec+odom_msg->header.stamp.nanosec*(1e-9)); idx++)
        ;

    if (init++ < SKIP)
    {
        baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = rclcpp::Time(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = baseRgt * Vector3d{benchmark[idx - 1].vx,
                                        benchmark[idx - 1].vy,
                                        benchmark[idx - 1].vz};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    pub_odom->publish(odometry);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path->publish(path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("benchmark_publisher");

    string csv_file = readParam<string>(n, "data_name");
    std::cout << "load ground truth " << csv_file << std::endl;
    FILE *f = fopen(csv_file.c_str(), "r");
    if (f==NULL)
    {
      RCUTILS_LOG_WARN("can't load ground truth; wrong path");
      //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
      return 0;
    }
    char tmp[10000];
    if (fgets(tmp, 10000, f) == NULL)
    {
        RCUTILS_LOG_WARN("can't load ground truth; no data available");
    }
    while (!feof(f))
        benchmark.emplace_back(f);
    fclose(f);
    benchmark.pop_back();
    RCUTILS_LOG_INFO("Data loaded: %d", (int)benchmark.size());

    pub_odom = n->create_publisher<nav_msgs::msg::Odometry>("odometry", 1000);
    pub_path = n->create_publisher<nav_msgs::msg::Path>("path", 1000);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom = n->create_subscription<nav_msgs::msg::Odometry>(
      "/vins_estimator/odometry", rclcpp::QoS(rclcpp::KeepLast(100)), odom_callback);
    
    rclcpp::spin(n);
    return 0;
}
