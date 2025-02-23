#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 5005
#define BUFFER_SIZE 1024

class UdpReceiverNode : public rclcpp::Node
{
    public:
        UdpReceiverNode() : Node("udp_receiver_node"){
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
            serv_addr.sin_port = htons(PORT);

            if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
                RCLCPP_ERROR(this->get_logger(), "Socket bind failed.");
                close(sockfd);
                rclcpp::shutdown();
            }

            RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", PORT);
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
        struct sockaddr_in serv_addr, cli_addr;
        socklen_t addrLen = sizeof(cli_addr);
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpReceiverNode>());
    rclcpp::shutdown();
    return 0;
}