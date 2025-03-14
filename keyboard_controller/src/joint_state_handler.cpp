#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include <thread>
#include <atomic>
#include <string>
#include <vector>

#include <termios.h>
#include <unistd.h>


struct Joint {
    std::string name;
    float position;
    Joint(std::string n, float p) : name(n), position(p) {}
};

class JointStateHandlerNode : public rclcpp::Node 
{
      public:
        JointStateHandlerNode() : Node("joint_state_handler") 
        {
            subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1000, std::bind(&JointStateHandlerNode::joint_state_read, this, std::placeholders::_1));

            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
            input_thread_ = std::thread(&JointStateHandlerNode::getInput, this);
        }

        ~JointStateHandlerNode() 
        {
            running = false;
            if(input_thread_.joinable()) {
                input_thread_.join();
            }
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
            
            joints.clear();
            
            for (size_t i=0; i<names.size(); i++)
            {
                joints.emplace_back(Joint(names[i].c_str(), positions[i]));
            }

            // for (size_t i=0; i<joints.size(); i++)
            // {
            //     RCLCPP_INFO(this->get_logger(), "'%s': %f", joints[i].name, joints[i].position);
            // }
        }

        int getch(void) 
        {
            int ch;
            struct termios oldt;
            struct termios newt;

            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;

            newt.c_lflag &= ~(ICANON | ECHO);
            newt.c_iflag |= IGNBRK;
            newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
            newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
            newt.c_cc[VMIN] = 1;
            newt.c_cc[VTIME] = 0;
            tcsetattr(fileno(stdin), TCSANOW, &newt);

            ch = getchar();

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

            return ch;
        }

        void getInput() 
        {
            float joint1_du_pos = 0.0;
            float joint2_dm_pos = 0.0;
            float joint2_mu_pos = 0.0;
            float joint3_dm_pos = 0.0;
            float joint3_mu_pos = 0.0;
            float joint4_dm_pos = 0.0;
            float joint4_mu_pos = 0.0;
            float joint5_dm_pos = 0.0;
            float joint5_mu_pos = 0.0;
            
            while(running) 
            {
                if (joints.empty())
                {
                    RCLCPP_WARN(this->get_logger(), "Waiting for joint states...");
                    continue;
                }
                
                int input;
                input = getch();
                
                for (size_t i=0; i<joints.size(); i++)
                {
                    std::string name;
                    float pos;

                    name = joints[i].name;
                    pos = joints[i].position;

                    
                    if (name == "joint1-du")
                    {joint1_du_pos = pos;}

                    else if (name == "joint2-dm")
                    {joint2_dm_pos = pos;}
                        
                    else if (name == "joint2-mu")
                    {joint2_mu_pos = pos;}

                    else if (name == "joint3-dm")
                    {joint3_dm_pos = pos;}

                    else if (name == "joint3-mu")
                    {joint3_mu_pos = pos;}

                    else if (name == "joint4-dm")
                    {joint4_dm_pos = pos;}

                    else if (name == "joint4-mu")
                    {joint4_mu_pos = pos;}

                    else if (name == "joint5-dm")
                    {joint5_dm_pos = pos;}

                    else if (name == "joint5-mu")
                    {joint5_mu_pos = pos;}
                }

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
                
                switch(input)
                {
                    case 97: //a
                        joint1_du_pos += stepVal;
                        break;

                    case 122: //z
                        joint1_du_pos -= stepVal;
                        break;

                    case 115: //s
                        joint2_dm_pos += stepVal;
                        break;

                    case 120: //x
                        joint2_dm_pos -= stepVal;
                        break;

                    case 100: //d
                        joint2_mu_pos += stepVal;
                        break;

                    case 99: //c
                        joint2_mu_pos -= stepVal;
                        break;

                    case 102: //f
                        joint3_dm_pos += stepVal;
                        break;

                    case 118: //v
                        joint3_dm_pos -= stepVal;
                        break;

                    case 103: //g
                        joint3_mu_pos += stepVal;
                        break;

                    case 98: //b
                        joint3_mu_pos -= stepVal;
                        break;

                    case 104: //h
                        joint4_dm_pos += stepVal;
                        break;

                    case 110: //n
                        joint4_dm_pos -= stepVal;
                        break;

                    case 106: //j
                        joint4_mu_pos += stepVal;
                        break;

                    case 109: //m
                        joint4_mu_pos -= stepVal;
                        break;

                    case 107: //k
                        joint5_dm_pos += stepVal;
                        break;

                    case 44: //,
                        joint5_dm_pos -= stepVal;
                        break;

                    case 108: //l
                        joint5_mu_pos += stepVal;
                        break;

                    case 46: //.
                        joint5_mu_pos -= stepVal;
                        break;

                    default:
                        joint1_du_pos += 0.0;
                        joint2_dm_pos += 0.0;
                        joint2_mu_pos += 0.0;
                        joint3_dm_pos += 0.0;
                        joint3_mu_pos += 0.0;
                        joint4_dm_pos += 0.0;
                        joint4_mu_pos += 0.0;
                        joint5_dm_pos += 0.0;
                        joint5_mu_pos += 0.0;
                }
                
                auto message = std_msgs::msg::Float64MultiArray();
                message.data = {joint1_du_pos, joint2_dm_pos, joint2_mu_pos, joint3_dm_pos, joint3_mu_pos, joint4_dm_pos, joint4_mu_pos, joint5_dm_pos, joint5_mu_pos};
                
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published: ");
                RCLCPP_INFO(this->get_logger(), "Joint 1		: '%.3f'", message.data[0]);
                RCLCPP_INFO(this->get_logger(), "Joint 2 (Down-Middle)	: '%.3f'", message.data[1]);
                RCLCPP_INFO(this->get_logger(), "Joint 2 (Middle-Up)	: '%.3f'", message.data[2]);
                RCLCPP_INFO(this->get_logger(), "Joint 3 (Down-Middle)	: '%.3f'", message.data[3]);
                RCLCPP_INFO(this->get_logger(), "Joint 3 (Middle-Up)	: '%.3f'", message.data[4]);
                RCLCPP_INFO(this->get_logger(), "Joint 4 (Down-Middle)	: '%.3f'", message.data[5]);
                RCLCPP_INFO(this->get_logger(), "Joint 4 (Middle-Up)	: '%.3f'", message.data[6]);
                RCLCPP_INFO(this->get_logger(), "Joint 5 (Down-Middle)	: '%.3f'", message.data[7]);
                RCLCPP_INFO(this->get_logger(), "Joint 5 (Middle-Up)	: '%.3f'", message.data[8]);
                
                if (input == 'q' || input == 'Q') {
                    running = false;
                }
            }
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        std::thread input_thread_;
        float stepVal = 0.15;
        std::atomic<bool> running{true};

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
        std::vector<Joint> joints;
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
