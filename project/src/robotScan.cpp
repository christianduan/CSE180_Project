// robot scan 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>

// this is where we laser scan and cout the data
class robotScan : public rclcpp::Node{
    public:
        pub = this->create_publisher<std_msgs::msg::Float32>("robotScan", 1000);

        sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&robotScan::scanCallback, this, std::placeholders::_1));

    private:
        void processScan(const sensor_msgs::msg::LaserSCan::SharedPtr msg){
            // process the scan and find the pillar
            // find the pillar
            std::vector<float> ranges = msg->ranges;
            std::vector<float>::iterator it = std::min_element(ranges.begin(), ranges.end());
            int index = std::distance(ranges.begin(), it);
            std::cout << "The pillar is at " << index << std::endl;
            std_msgs::msg::Float32 msg;
            msg.data = index;
            publisher_->publish(msg);
        }

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};


