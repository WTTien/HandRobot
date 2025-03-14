#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <thread>
#include <atomic>

#include <termios.h>
#include <unistd.h>

std::atomic<bool> running(true);

class KeyboardInputNode : public rclcpp::Node 
{
      public:
        KeyboardInputNode() : Node("keyboard_input_node") {
            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
            input_thread_ = std::thread(&KeyboardInputNode::getInput, this);
        }

        ~KeyboardInputNode() {
            running = false;
            if(input_thread_.joinable()) {
                input_thread_.join();
            }
        }

    private:
        int getch(void) {
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

        void getInput() {
            while(running) {
                int input;
                input = getch();

                float joint1 = 0.0;
                float joint2_dm = 0.0;
                float joint2_mu = 0.0;
                float joint3_dm = 0.0;
                float joint3_mu = 0.0;
                float joint4_dm = 0.0;
                float joint4_mu = 0.0;
                float joint5_dm = 0.0;
                float joint5_mu = 0.0;
                
                auto message = std_msgs::msg::Float64MultiArray();

                switch(input)
                {
                    case 97: //a
                        joint1 = stepVal;
                        break;

                    case 122: //z
                        joint1 = -stepVal;
                        break;

                    case 115: //s
                        joint2_dm = stepVal;
                        break;

                    case 120: //x
                        joint2_dm = -stepVal;
                        break;

                    case 100: //d
                        joint2_mu = stepVal;
                        break;

                    case 99: //c
                        joint2_mu = -stepVal;
                        break;

                    case 102: //f
                        joint3_dm = stepVal;
                        break;

                    case 118: //v
                        joint3_dm = -stepVal;
                        break;

                    case 103: //g
                        joint3_mu = stepVal;
                        break;

                    case 98: //b
                        joint3_mu = -stepVal;
                        break;

                    case 104: //h
                        joint4_dm = stepVal;
                        break;

                    case 110: //n
                        joint4_dm = -stepVal;
                        break;

                    case 106: //j
                        joint4_mu = stepVal;
                        break;

                    case 109: //m
                        joint4_mu = -stepVal;
                        break;

                    case 107: //k
                        joint5_dm = stepVal;
                        break;

                    case 44: //,
                        joint5_dm = -stepVal;
                        break;

                    case 108: //l
                        joint5_mu = stepVal;
                        break;

                    case 46: //.
                        joint5_mu = -stepVal;
                        break;

                    default:
                        joint1 = 0.0;
                        joint2_dm = 0.0;
                        joint2_mu = 0.0;
                        joint3_dm = 0.0;
                        joint3_mu = 0.0;
                        joint4_dm = 0.0;
                        joint4_mu = 0.0;
                        joint5_dm = 0.0;
                        joint5_mu = 0.0;
                }

                message.data = {joint1, joint2_dm, joint2_mu, joint3_dm, joint3_mu, joint4_dm, joint4_mu, joint5_dm, joint5_mu};
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published: ");
                RCLCPP_INFO(this->get_logger(), "Joint 1		: '%.1f'", message.data[0]);
                RCLCPP_INFO(this->get_logger(), "Joint 2 (Down-Middle)	: '%.1f'", message.data[1]);
                RCLCPP_INFO(this->get_logger(), "Joint 2 (Middle-Up)	: '%.1f'", message.data[2]);
                RCLCPP_INFO(this->get_logger(), "Joint 3 (Down-Middle)	: '%.1f'", message.data[3]);
                RCLCPP_INFO(this->get_logger(), "Joint 3 (Middle-Up)	: '%.1f'", message.data[4]);
                RCLCPP_INFO(this->get_logger(), "Joint 4 (Down-Middle)	: '%.1f'", message.data[5]);
                RCLCPP_INFO(this->get_logger(), "Joint 4 (Middle-Up)	: '%.1f'", message.data[6]);
                RCLCPP_INFO(this->get_logger(), "Joint 5 (Down-Middle)	: '%.1f'", message.data[7]);
                RCLCPP_INFO(this->get_logger(), "Joint 5 (Middle-Up)	: '%.1f'", message.data[8]);
                
                if (input == 'q' || input == 'Q') {
                    running = false;
                }
            }
        }

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        std::thread input_thread_;
        float stepVal = 1.5;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardInputNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
