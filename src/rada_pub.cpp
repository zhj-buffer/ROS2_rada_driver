#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rada_uart.hpp"

using namespace std::chrono_literals;
using namespace std;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RadaPublisher : public rclcpp::Node
{
  public:
    RadaPublisher()
    : Node("RadaPublisher"), count_(0)
    {
	  uart_init(fd);
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RadaPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
			auto message = std_msgs::msg::String();

			const char *w_buf = (const char *)"0x1";
			size_t w_len = sizeof(w_buf);
			int ret = -1;
			int fd = -1;
			char r_buf[11];
			bzero(r_buf,11);
//			r_buf[11]= '\0';

			ret = uart_write(fd,w_buf,w_len);
			if(ret == -1)
			{
					fprintf(stderr,"uart write failed!\n");
					exit(EXIT_FAILURE);
			}
			ret = uart_read(fd,r_buf,10);
			if(ret == -1)
			{
					fprintf(stderr,"uart read failed!\n");
					exit(EXIT_FAILURE);
			}
			printf(" r_buf: %02x %02x %02x %02x %02x %02x n: %d\n", r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], ret);


			r_buf[11] = '\0';
			string buf = r_buf;
			
			message.data = r_buf;
			RCLCPP_INFO(this->get_logger(), "Publishing: '%2x%2x%2x%2x%2x'", r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5]);
			publisher_->publish(message);
    }
	int fd;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadaPublisher>());
  rclcpp::shutdown();
  return 0;
}
