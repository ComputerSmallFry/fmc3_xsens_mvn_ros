#include <xsens_mvn_ros/XSensClient.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xsens_mvn_ros_msgs/msg/link_state_array.hpp>
#include <xsens_mvn_ros_msgs/msg/link_state.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("xsens_client");

    tf2_ros::TransformBroadcaster tf_broadcaster(node);

    // Parameters
    node->declare_parameter<std::string>("model_name", "skeleton");
    node->declare_parameter<std::string>("reference_frame", "world");
    node->declare_parameter<int>("udp_port", 8001);

    std::string model_name = node->get_parameter("model_name").as_string();
    std::string reference_frame = node->get_parameter("reference_frame").as_string();
    int xsens_udp_port = node->get_parameter("udp_port").as_int();

    // Publishers
    auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    auto link_state_publisher = node->create_publisher<xsens_mvn_ros_msgs::msg::LinkStateArray>("link_states", 10);
    auto com_publisher = node->create_publisher<geometry_msgs::msg::Point>("com", 10);

    std::shared_ptr<XSensClient> xsens_client_ptr;
    try
    {
        xsens_client_ptr = std::make_shared<XSensClient>(xsens_udp_port);
    }
    catch(const std::exception& err)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), err.what());
        return -1;
    }

    if(!xsens_client_ptr->init())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "XSens client initialization failed.");
        return -1;
    }

    rclcpp::Rate loop_rate(120);

    while(rclcpp::ok())
    {
        // Publish joint state
        if (joint_state_publisher->get_subscription_count() > 0)
        {
            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header.stamp = node->now();

            auto joints = xsens_client_ptr->getHumanData()->getJoints();
            for (auto joint_it = joints.begin(); joint_it != joints.end(); joint_it++)
            {
                joint_state_msg.name.push_back(model_name+"_"+joint_it->first+"_x");
                joint_state_msg.name.push_back(model_name+"_"+joint_it->first+"_y");
                joint_state_msg.name.push_back(model_name+"_"+joint_it->first+"_z");
                joint_state_msg.position.push_back(joint_it->second.state.angles[0]/180*3.1415);
                joint_state_msg.position.push_back(joint_it->second.state.angles[1]/180*3.1415);
                joint_state_msg.position.push_back(joint_it->second.state.angles[2]/180*3.1415);
            }

            joint_state_publisher->publish(joint_state_msg);
        }

        // Publish link tf and state
        xsens_mvn_ros_msgs::msg::LinkStateArray link_state_msg;
        auto links = xsens_client_ptr->getHumanData()->getLinks();
        for (auto link_it = links.begin(); link_it != links.end(); link_it++)
        {
            // Publish link tf
            if (!(link_it->second.state.orientation.x() == 0 && link_it->second.state.orientation.y() == 0
                && link_it->second.state.orientation.z() == 0 && link_it->second.state.orientation.w()))
            {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = node->now();
                transform_stamped.header.frame_id = reference_frame;
                transform_stamped.child_frame_id = model_name+"_"+link_it->first;
                transform_stamped.transform.translation.x = link_it->second.state.position[0];
                transform_stamped.transform.translation.y = link_it->second.state.position[1];
                transform_stamped.transform.translation.z = link_it->second.state.position[2];
                transform_stamped.transform.rotation.x = link_it->second.state.orientation.x();
                transform_stamped.transform.rotation.y = link_it->second.state.orientation.y();
                transform_stamped.transform.rotation.z = link_it->second.state.orientation.z();
                transform_stamped.transform.rotation.w = link_it->second.state.orientation.w();
                tf_broadcaster.sendTransform(transform_stamped);
            }

            if (link_state_publisher->get_subscription_count() > 0)
            {
                // Publish link state
                xsens_mvn_ros_msgs::msg::LinkState link_state;
                link_state.header.frame_id = link_it->first;
                link_state.header.stamp = node->now();
                link_state.pose.position = tf2::toMsg(link_it->second.state.position);
                link_state.pose.orientation = tf2::toMsg(link_it->second.state.orientation);

                Eigen::Matrix<double, 6, 1> link_twist;
                link_twist << link_it->second.state.velocity.linear, link_it->second.state.velocity.angular;
                link_state.twist = tf2::toMsg(link_twist);

                link_state.accel.linear.x = link_it->second.state.acceleration.linear[0];
                link_state.accel.linear.y = link_it->second.state.acceleration.linear[1];
                link_state.accel.linear.z = link_it->second.state.acceleration.linear[2];
                link_state.accel.angular.x = link_it->second.state.acceleration.angular[0];
                link_state.accel.angular.y = link_it->second.state.acceleration.angular[1];
                link_state.accel.angular.z = link_it->second.state.acceleration.angular[2];

                link_state_msg.states.push_back(link_state);
            }
        }

        if (link_state_publisher->get_subscription_count() > 0)
            link_state_publisher->publish(link_state_msg);

        if (com_publisher->get_subscription_count() > 0)
        {
            geometry_msgs::msg::Point com_msg;
            com_msg = tf2::toMsg(xsens_client_ptr->getHumanData()->getCOM());
            com_publisher->publish(com_msg);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
