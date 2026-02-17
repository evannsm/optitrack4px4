#include "optitrack4px4/communicator.hpp"
#include "optitrack4px4/utils.hpp"

// Free function NatNet callback — forwards to the Communicator member function
void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData *data, void *pUserData)
{
    static_cast<Communicator *>(pUserData)->process_frame(data);
}

// Constructor
Communicator::Communicator() : Node("optitrack_client")
{
    // Declare OptiTrack connection parameters
    this->declare_parameter<std::string>("connection_type", "Unicast");
    this->declare_parameter<std::string>("server_address", "192.168.1.113");
    this->declare_parameter<std::string>("local_address", "192.168.1.200");
    this->declare_parameter<std::string>("multicast_address", "239.255.42.99");
    this->declare_parameter<uint16_t>("server_command_port", 1510);
    this->declare_parameter<uint16_t>("server_data_port", 1511);

    // Declare frame/topic parameters (mirrors vicon4px4)
    this->declare_parameter<std::string>("namespace", "optitrack");
    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::string>("optitrack_frame", "optitrack");
    this->declare_parameter<std::vector<double>>("map_xyz", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("map_rpy", {0.0, 0.0, 0.0});
    this->declare_parameter<bool>("map_rpy_in_degrees", false);

    // Retrieve connection parameters
    this->get_parameter("connection_type", connection_type_);
    this->get_parameter("server_address", server_address_);
    this->get_parameter("local_address", local_address_);
    this->get_parameter("multicast_address", multicast_address_);
    this->get_parameter("server_command_port", server_command_port_);
    this->get_parameter("server_data_port", server_data_port_);

    // Retrieve frame/topic parameters
    this->get_parameter("namespace", ns_name);
    this->get_parameter("world_frame", world_frame);
    this->get_parameter("optitrack_frame", optitrack_frame);
    this->get_parameter("map_xyz", map_xyz);
    this->get_parameter("map_rpy", map_rpy);
    this->get_parameter("map_rpy_in_degrees", map_rpy_in_degrees);

    // Convert degrees to radians if needed
    if (map_rpy_in_degrees)
    {
        for (unsigned int i = 0; i < map_rpy.size(); i++)
        {
            map_rpy[i] = map_rpy[i] * M_PI / 180.0;
        }
    }

    // Create NatNet client and register frame callback
    client_ = new NatNetClient();
    client_->SetFrameReceivedCallback(process_frame_callback, this);

    // Publish static transform from world_frame to optitrack_frame
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_static_transform();

    // Initialize the dynamic tf2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

Communicator::~Communicator()
{
    if (client_)
    {
        delete client_;
        client_ = nullptr;
    }
}

// Publish the static transform from world_frame (map) to optitrack_frame (optitrack)
void Communicator::publish_static_transform()
{
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = world_frame;
    static_tf.child_frame_id = optitrack_frame;

    static_tf.transform.translation.x = map_xyz[0];
    static_tf.transform.translation.y = map_xyz[1];
    static_tf.transform.translation.z = map_xyz[2];
    tf2::Quaternion q;
    q.setRPY(
        map_rpy[0],
        map_rpy[1],
        map_rpy[2]);
    static_tf.transform.rotation.x = q.x();
    static_tf.transform.rotation.y = q.y();
    static_tf.transform.rotation.z = q.z();
    static_tf.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(static_tf);

    RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s",
                world_frame.c_str(), optitrack_frame.c_str());
}

// Connect to the OptiTrack / Motive server via NatNet
bool Communicator::connect()
{
    RCLCPP_INFO(this->get_logger(), "Connecting to OptiTrack server at %s ...",
                server_address_.c_str());

    // Configure connection parameters
    if (connection_type_ == "Multicast")
    {
        client_params_.connectionType = ConnectionType::ConnectionType_Multicast;
        client_params_.multicastAddress = multicast_address_.c_str();
    }
    else
    {
        client_params_.connectionType = ConnectionType::ConnectionType_Unicast;
    }

    client_params_.serverAddress = server_address_.c_str();
    client_params_.localAddress = local_address_.c_str();
    client_params_.serverCommandPort = server_command_port_;
    client_params_.serverDataPort = server_data_port_;

    // Attempt connection
    int retries = 0;
    while (rclcpp::ok())
    {
        ErrorCode result = client_->Connect(client_params_);
        if (result == ErrorCode_OK)
        {
            break;
        }
        retries++;
        RCLCPP_WARN(this->get_logger(), "Connect failed (attempt %d), retrying...", retries);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!rclcpp::ok())
    {
        RCLCPP_ERROR(this->get_logger(), "Shutdown requested before connection established.");
        return false;
    }

    // Get server description
    memset(&server_description_, 0, sizeof(server_description_));
    client_->GetServerDescription(&server_description_);

    if (!server_description_.HostPresent)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to connect to server. Host not present.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to server: %s (ver. %d.%d.%d.%d)",
                server_description_.szHostApp,
                server_description_.HostAppVersion[0],
                server_description_.HostAppVersion[1],
                server_description_.HostAppVersion[2],
                server_description_.HostAppVersion[3]);

    RCLCPP_INFO(this->get_logger(), "NatNet Version: %d.%d.%d.%d",
                server_description_.NatNetVersion[0],
                server_description_.NatNetVersion[1],
                server_description_.NatNetVersion[2],
                server_description_.NatNetVersion[3]);

    // Query frame rate
    void *pResult;
    int nBytes = 0;
    if (client_->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK)
    {
        float fRate = *(static_cast<float *>(pResult));
        RCLCPP_INFO(this->get_logger(), "Mocap Framerate: %.2f", fRate);
    }

    // Get data descriptions to build rigid body ID → name mapping
    if (client_->GetDataDescriptionList(&data_descriptions_) == ErrorCode_OK && data_descriptions_)
    {
        for (int i = 0; i < data_descriptions_->nDataDescriptions; i++)
        {
            if (data_descriptions_->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                sRigidBodyDescription *rb_desc =
                    data_descriptions_->arrDataDescriptions[i].Data.RigidBodyDescription;
                id_to_name_[rb_desc->ID] = std::string(rb_desc->szName);
                RCLCPP_INFO(this->get_logger(), "Rigid body [%d]: %s",
                            rb_desc->ID, rb_desc->szName);
            }
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unable to retrieve data descriptions.");
    }

    RCLCPP_INFO(this->get_logger(), "Initialization complete. Streaming data...");
    return true;
}

// Disconnect from the OptiTrack server
bool Communicator::disconnect()
{
    RCLCPP_INFO(this->get_logger(), "Disconnecting from OptiTrack server...");

    void *response;
    int nBytes;
    if (client_->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK)
    {
        client_->Disconnect();
        RCLCPP_INFO(this->get_logger(), "Successfully disconnected.");
        return true;
    }
    else
    {
        // Still try to disconnect even if the message fails
        client_->Disconnect();
        RCLCPP_WARN(this->get_logger(), "Disconnect message failed, forced disconnect.");
        return true;
    }
}

// Process a frame of mocap data (called from NatNet callback thread)
void Communicator::process_frame(sFrameOfMocapData *data)
{
    map<string, Publisher>::iterator pub_it;

    for (int i = 0; i < data->nRigidBodies; i++)
    {
        sRigidBodyData &rb = data->RigidBodies[i];

        // Look up rigid body name from data descriptions
        std::string rb_name;
        auto name_it = id_to_name_.find(rb.ID);
        if (name_it != id_to_name_.end())
        {
            rb_name = name_it->second;
        }
        else
        {
            rb_name = "rb_" + std::to_string(rb.ID);
        }

        // Build a TF message for this rigid body
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = optitrack_frame;
        tf_msg.child_frame_id = rb_name + "_" + rb_name;

        // OptiTrack Y-up (right-handed) → NED conversion
        // OptiTrack default: X = left, Y = up, Z = forward (right-handed: X × Y = Z)
        // NED:               X = forward (North), Y = right (East), Z = down
        // Position mapping (already in meters)
        tf_msg.transform.translation.x = rb.z;    // forward → North
        tf_msg.transform.translation.y = -rb.x;   // -left = right → East
        tf_msg.transform.translation.z = -rb.y;   // -up = down → Down

        // Quaternion Y-up → NED: remap axes consistently with position
        tf_msg.transform.rotation.x = rb.qz;      // forward axis → X
        tf_msg.transform.rotation.y = -rb.qx;     // -left axis → Y
        tf_msg.transform.rotation.z = -rb.qy;     // -up axis → Z
        tf_msg.transform.rotation.w = rb.qw;


        // Publish the position data
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (lock.owns_lock())
        {
            pub_it = pub_map.find(rb_name);
            if (pub_it != pub_map.end())
            {
                Publisher &pub = pub_it->second;

                if (pub.is_ready)
                {
                    // Build a PoseStamped in the optitrack frame
                    geometry_msgs::msg::PoseStamped optitrack_pose_msg;
                    optitrack_pose_msg.header = tf_msg.header;

                    optitrack_pose_msg.pose.position.x = tf_msg.transform.translation.x;
                    optitrack_pose_msg.pose.position.y = tf_msg.transform.translation.y;
                    optitrack_pose_msg.pose.position.z = tf_msg.transform.translation.z;

                    optitrack_pose_msg.pose.orientation = tf_msg.transform.rotation;

                    // Update timestamp of static transform
                    static_tf.header.stamp = tf_msg.header.stamp;

                    // Transform the pose to the global (map) frame
                    geometry_msgs::msg::PoseStamped global_pose_msg;
                    tf2::doTransform(optitrack_pose_msg, global_pose_msg, static_tf);

                    // Publish the transformed pose
                    pub.publish(global_pose_msg);
                }
            }
            else
            {
                // Create a publisher if it doesn't exist
                std::string key = rb_name;
                if (pending_publishers.find(key) == pending_publishers.end())
                {
                    pending_publishers.insert(key);
                    lock.unlock();
                    create_publisher(rb_name);
                }
            }
        }

        // Broadcast the transform
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

// Create a publisher for a rigid body
void Communicator::create_publisher(const string rb_name)
{
    std::thread(&Communicator::create_publisher_thread, this, rb_name).detach();
}

// Thread function to create a publisher
void Communicator::create_publisher_thread(const string rb_name)
{
    // Topic: /{namespace}/{rb_name}/{rb_name}
    std::string topic_name = ns_name + "/" + rb_name + "/" + rb_name;

    RCLCPP_INFO(this->get_logger(), "Creating publisher for rigid body '%s' on topic '%s'",
                rb_name.c_str(), topic_name.c_str());

    std::lock_guard<std::mutex> lock(mutex_);
    pub_map.insert(std::map<std::string, Publisher>::value_type(rb_name, Publisher(topic_name, this)));
    pending_publishers.erase(rb_name);
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    // Connect to the OptiTrack server
    if (!node->connect())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect to OptiTrack server.");
        rclcpp::shutdown();
        return 1;
    }

    // NatNet delivers frames via callback, so just spin
    rclcpp::spin(node);

    // Disconnect and shut down
    node->disconnect();
    rclcpp::shutdown();
    return 0;
}
