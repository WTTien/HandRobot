#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>

class JointStateHandlerNode : public rclcpp::Node 
{
  	public:
		JointStateHandlerNode() : Node("joint_state_handler") 
		{
            subscriber_ = this->create_subscription<std_msgs::msg::String>("/camera_data", 10, std::bind(&JointStateHandlerNode::joint_control, this, std::placeholders::_1));
			publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
		}

	private:
		void joint_control(const std_msgs::msg::String::SharedPtr in_msg)
        {
            std::istringstream ss(in_msg->data);
            std::string token;
            std::vector<double> positions;

            while (std::getline(ss, token, ',')) {
                positions.push_back(std::stod(token));
            }

            std::transform(positions.begin(), positions.end(), positions.begin(), [](double deg) {
                return deg * M_PI / 180.0;
            });
            
            float joint1_du_pos = positions[0];
			float joint2_dm_pos = positions[1];
			float joint2_mu_pos = 0.0;
			float joint3_dm_pos = positions[2];
			float joint3_mu_pos = 0.0;
			float joint4_dm_pos = positions[3];
			float joint4_mu_pos = 0.0;
			float joint5_dm_pos = positions[4];
			float joint5_mu_pos = 0.0;
            
            auto out_msg = std_msgs::msg::Float64MultiArray();
            out_msg.data = {joint1_du_pos, joint2_dm_pos, joint2_mu_pos, joint3_dm_pos, joint3_mu_pos, joint4_dm_pos, joint4_mu_pos, joint5_dm_pos, joint5_mu_pos};
            
            publisher_->publish(out_msg);
				

            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "Joint Info:");
            RCLCPP_INFO(this->get_logger(), "Joint 1		: %f", joint1_du_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 2 (Down-Middle)	: %f", joint2_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 2 (Middle-Up)	: %f", joint2_mu_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 3 (Down-Middle)	: %f", joint3_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 3 (Middle-Up)	: %f", joint3_mu_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 4 (Down-Middle)	: %f", joint4_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 4 (Middle-Up)	: %f", joint4_mu_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 5 (Down-Middle)	: %f", joint5_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 5 (Middle-Up)	: %f", joint5_mu_pos);
			
        }

		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) 
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JointStateHandlerNode>());
	rclcpp::shutdown();
	return 0;
}
