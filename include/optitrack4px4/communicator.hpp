#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetCAPI.h"

#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <mutex>
#include <chrono>
#include <string>
#include <set>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;

// Free function callback for NatNet frame delivery
void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData *data, void *pUserData);

// Main Node class
class Communicator : public rclcpp::Node
{
private:
    NatNetClient *client_;
    sNatNetClientConnectParams client_params_;
    sServerDescription server_description_;
    sDataDescriptions *data_descriptions_ = nullptr;

    // Rigid body ID → name mapping (populated from data descriptions)
    std::map<int, std::string> id_to_name_;

    // Connection parameters
    string connection_type_;
    string server_address_;
    string local_address_;
    string multicast_address_;
    uint16_t server_command_port_;
    uint16_t server_data_port_;

    string ns_name;
    map<string, Publisher> pub_map;
    std::mutex mutex_;
    std::set<std::string> pending_publishers;

    geometry_msgs::msg::TransformStamped static_tf;
    string world_frame;
    string optitrack_frame;
    vector<double> map_xyz;
    vector<double> map_rpy;
    bool map_rpy_in_degrees;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void publish_static_transform();

public:
    Communicator();
    ~Communicator();

    // Connect to the OptiTrack server
    bool connect();

    // Disconnect from the OptiTrack server
    bool disconnect();

    // Process a frame of mocap data (called from NatNet callback thread)
    void process_frame(sFrameOfMocapData *data);

    // Create a publisher for a rigid body
    void create_publisher(const string rb_name);
    void create_publisher_thread(const string rb_name);
};

#endif // COMMUNICATOR_HPP
