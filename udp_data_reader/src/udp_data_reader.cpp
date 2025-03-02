#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define BUFFER_SIZE 1024

class UdpReceiverNode : public rclcpp::Node
{
    public:
        UdpReceiverNode(int port) : Node("udp_receiver_node"), port_(port) {
            publisher_ = this->create_publisher<std_msgs::msg::String>("camera_data", 10);
            setup_socket();
            receive_data();
        }

        ~UdpReceiverNode(){
            close(sockfd);
        }

    private:
        void setup_socket(){
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0){
                RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
                rclcpp::shutdown();
            }

            memset(&serv_addr, 0, sizeof(serv_addr));

            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = INADDR_ANY;
            serv_addr.sin_port = htons(port_);

            if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
                RCLCPP_ERROR(this->get_logger(), "Socket bind failed.");
                close(sockfd);
                rclcpp::shutdown();
            }

            RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", port_);
        }

        void receive_data(){
            char buffer[BUFFER_SIZE];

            while (rclcpp::ok()){
                int bytesReceived = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&cli_addr, &addrLen);
                if (bytesReceived < 0){
                    RCLCPP_ERROR(this->get_logger(), "Error receiving data.");
                    continue;
                }

                // buffer[bytesReceived] = '\0';
                std_msgs::msg::String msg;
                msg.data = buffer;
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
            }
        }
        
        int sockfd;
        int port_;
        struct sockaddr_in serv_addr, cli_addr;
        socklen_t addrLen = sizeof(cli_addr);
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cerr << "Error: Missing port number argument. \n";
        std::cout << "Usage: ros2 run handrobot_ros2_control udp_data_reader <port_number>\n"
                    << "Example: ros2 run handrobot_ros2_control udp_data_reader 5005\n";
        return 1;
    }

    int port = atoi(argv[1]);
    if (port <= 0 || port > 65535)
    {
        std::cerr << "Error: Invalid port number. Port number must be smaller than 65535. \n";
        return 1;
    }

    rclcpp::spin(std::make_shared<UdpReceiverNode>(port));
    rclcpp::shutdown();
    return 0;
}