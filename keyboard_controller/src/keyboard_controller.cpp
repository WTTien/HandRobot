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
			publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);
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

				auto message = std_msgs::msg::String();
				if (input == 97) {
					message.data = "YEET";
				}
				else {
					input = static_cast<char>(input);
					message.data = std::string(1, input);
				}

				publisher_->publish(message);
				RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());

				if (input == 'q' || input == 'Q') {
					running = false;
				}
			}
		}

		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		std::thread input_thread_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<KeyboardInputNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}