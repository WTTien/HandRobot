#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>

class CameraControllerNode : public rclcpp::Node 
{
  	public:
		CameraControllerNode() : Node("camera_controller") 
		{
            subscriber_ = this->create_subscription<std_msgs::msg::String>("/camera_data", 10, std::bind(&CameraControllerNode::joint_control, this, std::placeholders::_1));
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
            
            float joint1_bd_pos = positions[0];
            float joint1_du_pos = positions[1];
            float joint2_bd_pos = positions[2];
			float joint2_dm_pos = positions[3];
			float joint2_mu_pos = positions[4];
            float joint3_bd_pos = positions[5];
			float joint3_dm_pos = positions[6];
			float joint3_mu_pos = positions[7];
            float joint4_bd_pos = positions[8];
			float joint4_dm_pos = positions[9];
			float joint4_mu_pos = positions[10];
            float joint5_bd_pos = positions[11];
			float joint5_dm_pos = positions[12];
			float joint5_mu_pos = positions[13];
            
            auto out_msg = std_msgs::msg::Float64MultiArray();
            out_msg.data = {joint1_bd_pos, joint1_du_pos, 
                            joint2_bd_pos, joint2_dm_pos, joint2_mu_pos, 
                            joint3_bd_pos, joint3_dm_pos, joint3_mu_pos, 
                            joint4_bd_pos, joint4_dm_pos, joint4_mu_pos, 
                            joint5_bd_pos, joint5_dm_pos, joint5_mu_pos};
            
            publisher_->publish(out_msg);
				

            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "Joint Info:");
            RCLCPP_INFO(this->get_logger(), "Joint 1 (Base-Down)	: %f", joint1_bd_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 1 (Down-Up)		: %f", joint1_du_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 2 (Base-Down)	: %f", joint2_bd_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 2 (Down-Middle)	: %f", joint2_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 2 (Middle-Up)	: %f", joint2_mu_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 3 (Base-Down)	: %f", joint3_bd_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 3 (Down-Middle)	: %f", joint3_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 3 (Middle-Up)	: %f", joint3_mu_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 4 (Base-Down)	: %f", joint4_bd_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 4 (Down-Middle)	: %f", joint4_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 4 (Middle-Up)	: %f", joint4_mu_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 5 (Base-Down)	: %f", joint5_bd_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 5 (Down-Middle)	: %f", joint5_dm_pos);
            RCLCPP_INFO(this->get_logger(), "Joint 5 (Middle-Up)	: %f", joint5_mu_pos);
			
        }

		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char *argv[]) 
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraControllerNode>());
	rclcpp::shutdown();
	return 0;
}
