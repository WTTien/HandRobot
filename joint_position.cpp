#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <vector>

struct Joint {
    std::string name;
    float position;
    Joint(std::string n, float p) : name(n), position(p) {}
};

class JointStateSubscriber : public rclcpp::Node
{
    public:
        JointStateSubscriber() : Node("JointState_Subscriber")
        {
            subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1000, std::bind(&JointStateSubscriber::joint_state_read, this, std::placeholders::_1));
        }

    private:
        void joint_state_read(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::vector<std::string> names;
            std::vector<double> positions;

            names = msg->name;
            positions = msg->position;

            if(names.size() != positions.size()) 
            {
                std::cerr << "Error: 'name' and 'position' vectors not having same size!" << std::endl;
                return;
            }

            for (size_t i=0; i<names.size(); i++)
            {
                joints.emplace_back(Joint(names[i], positions[i]));
            }

            for (size_t i=0; i<joints.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "'%s': %f", joints[i].name.c_str(), joints[i].position);
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
        std::vector<Joint> joints;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}