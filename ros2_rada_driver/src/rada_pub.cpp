#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/thread/mutex.hpp>
#include "rclcpp/rclcpp.hpp"
#include "ros2_rada_msg/msg/rada.hpp"
#include "rada_uart.hpp"

using namespace std::chrono_literals;
using namespace std;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RadaPublisher : public rclcpp::Node
{
  public:
    RadaPublisher()
    : Node("RadaPublisher")
    {

      unsigned char  w_buf[8] = {0x01, 0x06, 0x02, 0x02, 0x00, 0x01, 0xe8, 0x72};
      size_t w_len = sizeof(w_buf);
      unsigned char r_buf[10] = {0};
      int ret = -1;

#if 1
      fd1 = uart_open("/dev/ttyUSB4");
      uart_setup(fd1, B9600);
      ret = uart_write(fd1,w_buf,w_len);
      if(ret == -1)
      {
	      fprintf(stderr,"uart write failed!\n");
	      exit(EXIT_FAILURE);
      }
      ret = uart_read(fd1,r_buf,w_len);
      if(ret == -1)
      {
	      fprintf(stderr,"uart read failed!\n");
	      exit(EXIT_FAILURE);
      }

      publisher1_ = this->create_publisher<ros2_rada_msg::msg::Rada>("rada1", 10);
      timer1_ = this->create_wall_timer(
		      100ms, std::bind(&RadaPublisher::timer_callback1, this));
#endif
#if 1
      fd2 = uart_open("/dev/ttyUSB5");
      uart_setup(fd2, B9600);
      ret = uart_write(fd2,w_buf,w_len);
      if(ret == -1)
      {
	      fprintf(stderr,"uart write failed!\n");
	      exit(EXIT_FAILURE);
      }
      ret = uart_read(fd2,r_buf,w_len);
      if(ret == -1)
      {
	      fprintf(stderr,"uart read failed!\n");
	      exit(EXIT_FAILURE);
      }
      publisher2_ = this->create_publisher<ros2_rada_msg::msg::Rada>("rada2", 10);
      timer2_ = this->create_wall_timer(
		      100ms, std::bind(&RadaPublisher::timer_callback2, this));
#endif
    }

    ~RadaPublisher() {
	uart_close(fd1);
	uart_close(fd2);
    }


  private:
    void timer_callback1()
    {

	    auto message = ros2_rada_msg::msg::Rada();

	    mutex1.lock();
	    const char *w_buf = (const char *)"11111";
	    size_t w_len = sizeof(w_buf);
	    int ret = -1;
	    unsigned char r_buf[10]={0};
	    ret = uart_read(fd1,r_buf,10);
	    if(ret == -1)
	    {
		    fprintf(stderr,"uart read failed!\n");
		    exit(EXIT_FAILURE);
	    }
	    message.r1 = (r_buf[1] << 8) + r_buf[2];
	    message.r2 = (r_buf[3] << 8) + r_buf[4];
	    message.r3 = (r_buf[5] << 8) + r_buf[6];
	    message.r4 = (r_buf[7] << 8) + r_buf[8];
	    message.crc = (r_buf[0] + r_buf[1] + r_buf[2] + r_buf[3] + r_buf[4] + r_buf[5] + r_buf[6] + r_buf[7] + r_buf[8]) & 0x00ff;

	    if (message.crc == r_buf[9]) {
		    RCLCPP_INFO(this->get_logger()," Raw1: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  n: %d ", r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5],r_buf[6], r_buf[7], r_buf[8], r_buf[9] ,ret);
		    RCLCPP_INFO(this->get_logger(),"CRC1: %04x\n", (r_buf[0] + r_buf[1] + r_buf[2] + r_buf[3] + r_buf[4] + r_buf[5] + r_buf[6] + r_buf[7] + r_buf[8]) & 0x00ff);
		    publisher1_->publish(message);
	    }
	    else {
		    RCLCPP_INFO(this->get_logger()," Raw1: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  n: %d ", r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5],r_buf[6], r_buf[7], r_buf[8], r_buf[9] ,ret);
		    RCLCPP_INFO(this->get_logger(),"CRC1: %04x\n", (r_buf[0] + r_buf[1] + r_buf[2] + r_buf[3] + r_buf[4] + r_buf[5] + r_buf[6] + r_buf[7] + r_buf[8]) & 0x00ff);
		    RCLCPP_INFO(this->get_logger(), "1 Rada CRC Wrong!\n");
	    }
	    mutex1.unlock();

    }
    void timer_callback2()
    {

	    auto message = ros2_rada_msg::msg::Rada();

	    mutex2.lock();
	    const char *w_buf = (const char *)"11111";
	    size_t w_len = sizeof(w_buf);
	    int ret = -1;
	    unsigned char r_buf[10] = {0};
	    ret = uart_read(fd2,r_buf,10);
	    if(ret == -1)
	    {
		    fprintf(stderr,"uart read failed!\n");
		    exit(EXIT_FAILURE);
	    }


	    message.r1 = (r_buf[1] << 8) + r_buf[2];
	    message.r2 = (r_buf[3] << 8) + r_buf[4];
	    message.r3 = (r_buf[5] << 8) + r_buf[6];
	    message.r4 = (r_buf[7] << 8) + r_buf[8];
	    message.crc = (r_buf[0] + r_buf[1] + r_buf[2] + r_buf[3] + r_buf[4] + r_buf[5] + r_buf[6] + r_buf[7] + r_buf[8]) & 0x00ff;

	    if (message.crc == r_buf[9]) {
		    RCLCPP_INFO(this->get_logger()," Raw2: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  n: %d :", r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5],r_buf[6], r_buf[7], r_buf[8], r_buf[9] ,ret);
		    RCLCPP_INFO(this->get_logger(),"CRC2: %04x\n", (r_buf[0] + r_buf[1] + r_buf[2] + r_buf[3] + r_buf[4] + r_buf[5] + r_buf[6] + r_buf[7] + r_buf[8]) & 0x00ff);
		    publisher2_->publish(message);
	    }
	    else {
		    RCLCPP_INFO(this->get_logger(), "2 Rada CRC Wrong!\n");
		    RCLCPP_INFO(this->get_logger()," Raw2: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  n: %d :", r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5],r_buf[6], r_buf[7], r_buf[8], r_buf[9],  ret);
		    RCLCPP_INFO(this->get_logger(),"CRC2: %04x\n", (r_buf[0] + r_buf[1] + r_buf[2] + r_buf[3] + r_buf[4] + r_buf[5] + r_buf[6] + r_buf[7] + r_buf[8]) & 0x00ff);
	    }
	    mutex2.unlock();

    }

    int fd1;
    int fd2;
    boost::mutex mutex1;
    boost::mutex mutex2;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::Publisher<ros2_rada_msg::msg::Rada>::SharedPtr publisher1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<ros2_rada_msg::msg::Rada>::SharedPtr publisher2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadaPublisher>());
  rclcpp::shutdown();
  return 0;
}
