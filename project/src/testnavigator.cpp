#include <rclcpp/rclcpp.hpp>
#include <navigation/navigation.hpp>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>
#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <random> 
// This File will test the navigation library with a robot that has a laser scanner. The robot is to move around the map that has 9 pillars set and 1 placed at random. The robot will then try to seek the random pillar and move to it to be able to give cordinates of the pillar.

// misc
double x, y;
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> dis(-0.25, 0.25);
double randNumX = dis(gen);
double randNumY = dis(gen);
bool tl = false; bool tm = false; bool tr = true; bool ml = false; bool mm = false; 
bool mr = false; bool bl = false; bool bm = false; bool br = false;
double tl_x = -1; double tl_y = 1; double tm_x = 0; double tm_y = 1; double tr_x = 1 + randNumX; double tr_y = 1 + randNumY;
double ml_x = -1; double ml_y = 0; double mm_x = 0; double mm_y = 0; double mr_x = 1; double mr_y = 0;
double bl_x = -1; double bl_y = -1; double bm_x = 0; double bm_y = -1; double br_x = 1; double br_y = -1;

// amcl pose 
void amclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
  x = msg->pose.pose.position.x + randNumX;
  y = -msg->pose.pose.position.y + randNumY;
}

// initialize the scanner 
class robotScan : public rclcpp::Node {
  public:
    robotScan() : Node("robotScan"){
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("robotScan", 1000);

      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&robotScan::processScan, this, std::placeholders::_1));
    }

  private:
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg){
      // process the scan and find the pillar
      // find the pillar
      std::vector<float> ranges = scanMsg->ranges;
      std::vector<float>::iterator it = std::min_element(ranges.begin(), ranges.end());
      int index = std::distance(ranges.begin(), it);
      std::cout << "The pillar is at " << index << std::endl;
      std_msgs::msg::Float32 robotFloatMsg;
      robotFloatMsg.data = index;
      publisher_->publish(robotFloatMsg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv){
  
  rclcpp::init(argc, argv);
  Navigator navigator(true);
  int pillarHeight = 0.25;

  // amcl pose subscriber
  auto node = std::make_shared<rclcpp::Node>("amcl_sub");
  auto subscription = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),std::bind(&amclPose, std::placeholders::_1));

  // initialize the pillars
  geometry_msgs::msg::Pose::SharedPtr p0 = std::make_shared<geometry_msgs::msg::Pose>();
  p0->position.x = 1.1;
  p0->position.y = -1.1;
  p0->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p1 = std::make_shared<geometry_msgs::msg::Pose>();
  p1->position.x = 1.1;
  p1->position.y = 0;
  p1->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p2 = std::make_shared<geometry_msgs::msg::Pose>();
  p2->position.x = 1.1;
  p2->position.y = 1.1;
  p2->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p3 = std::make_shared<geometry_msgs::msg::Pose>();
  p3->position.x = 0;
  p3->position.y = -1.1;
  p3->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p4 = std::make_shared<geometry_msgs::msg::Pose>();
  p4->position.x = 0;
  p4->position.y = 0;
  p4->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p5 = std::make_shared<geometry_msgs::msg::Pose>();
  p5->position.x = 0;
  p5->position.y = 1.1;
  p5->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p6 = std::make_shared<geometry_msgs::msg::Pose>();
  p6->position.x = -1.1;
  p6->position.y = -1.1;
  p6->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p7 = std::make_shared<geometry_msgs::msg::Pose>();
  p7->position.x = -1.1;
  p7->position.y = 0;
  p7->position.z = pillarHeight;

  geometry_msgs::msg::Pose::SharedPtr p8 = std::make_shared<geometry_msgs::msg::Pose>();
  p8->position.x = -1.1;
  p8->position.y = 1.1;
  p8->position.z = pillarHeight;  

  // initialize the map
  std::shared_ptr<nav2_msgs::msg::Costmap> globalCostmap = navigator.GetGlobalCostmap();
  std::shared_ptr<nav2_msgs::msg::Costmap> localCostmap = navigator.GetLocalCostmap();

  // initialize the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  navigator.WaitUntilNav2Active();

  geometry_msgs::msg::Pose::SharedPtr goal = std::make_shared<geometry_msgs::msg::Pose>();

  // move the robot to different positions around the map
  goal->position.x = -1;
  goal->position.y = -1.75;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }

  goal->position.x = 1.75;
  goal->position.y = -0.5;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

    // if controller server passes a new route to the robot have it save the coordinates

  }

  goal->position.x = -2;
  goal->position.y = -0.5;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }

  goal->position.x = -2;
  goal->position.y = 0.5;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }

  goal->position.x = 1.75;
  goal->position.y = 0.5;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }
  
  goal->position.x = 1.75;
  goal->position.y = 1.5;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }

  goal->position.x = 0.5;
  goal->position.y = 1.75;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }
  
  goal->position.x = 0.5;
  goal->position.y = -1.75;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }

  goal->position.x = -0.5;
  goal->position.y = -1.75;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }
  
  goal->position.x = -0.5;
  goal->position.y = 1.75;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }
  goal->position.x = -2;
  goal->position.y = -0.5;
  goal->orientation.w = 1;
  navigator.GoToPose(goal);
  while (!navigator.IsTaskComplete()){
    auto feedback_ptr = navigator.GetFeedback();
    auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
    std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;

  }

  if(tl == true){
    std::cout << "A square pillar is at " << tl_x << ", " << tl_y << std::endl;
  }
  if(tm == true){
    std::cout << "A square pillar is at " << tm_x << ", " << tm_y << std::endl;
  }
  if(tr == true){
    std::cout << "A square pillar is at " << tr_x << ", " << tr_y << std::endl;
  }
  if(ml == true){
    std::cout << "A square pillar is at " << ml_x << ", " << ml_y << std::endl;
  }
  if(mm == true){
    std::cout << "A square pillar is at " << mm_x << ", " << mm_y << std::endl;
  }
  if(mr == true){
    std::cout << "A square pillar is at " << mr_x << ", " << mr_y << std::endl;
  }
  if(bl == true){
    std::cout << "A square pillar is at " << bl_x << ", " << bl_y << std::endl;
  }
  if(bm == true){
    std::cout << "A square pillar is at " << bm_x << ", " << bm_y << std::endl;
  }
  if(br == true){
    std::cout << "A square pillar is at " << br_x << ", " << br_y << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}