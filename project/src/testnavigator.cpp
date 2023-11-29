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
double randNum = dis(gen);

// amcl pose 
void amclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
  x = msg->pose.pose.position.x + randNum;
  y = -msg->pose.pose.position.y + randNum;

  std::cout << "The pillar is at " << x << ", " << y << std::endl;
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
  double tolerance = 0.25;

  // amcl pose subscriber
  auto node = std::make_shared<rclcpp::Node>("amcl_sub");
  auto subscription = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),std::bind(&amclPose, std::placeholders::_1));

  double amcX = x;
  double amcY = y;

  // initialize the scanner
  // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robotScan");
  // robotScan scan;
  // rclcpp::executors::SingleThreadedExecutor exec;
  // exec.add_node(node);
  // exec.add_node(scan);
  // exec.spin();

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
  
  if(std::abs(amcX - amcY) <= tolerance){
    rclcpp::spin(node);
  }
  else{
    std::cout << "The pillar is at " << amcX << ", " << amcY << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}


/*Misc

 // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("map_sub");
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, mapcallBAck);

    // initialize the laser scanner
    // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robotScan");
    // robotScan scan;
    // rclcpp::executors::SingleThreadedExecutor exec;
    // exec.add_node(node);
    // exec.add_node(scan);
    // exec.spin();


     
void navigator::changemap(const std::string& map_filepath) {
  using namespace std::chrono_literals;
  while ( ! change_map_srv->wait_for_service(1s) )
    if ( debug )
      rclcpp_info(get_logger(),"change map service not available, waiting...");
  auto request = std::make_shared<nav2_msgs::srv::loadmap::request>();
  request->map_url = map_filepath;
  auto future_change_map = change_map_srv->async_send_request(request);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_change_map);
  auto result = future_change_map.get();
  if (debug) {
    if (result->result != result->result_success)
      rclcpp_info(get_logger(),"change map request failed");
    else
      rclcpp_info(get_logger(),"change map request was successful");
  }
  if (result->result == result->result_success)
    status = rclcpp_action::resultcode::succeeded;
  else
    status = rclcpp_action::resultcode::canceled;
}

  navigator.Spin( );
  while(!navigator.IsTaskComplete()){
      auto feedback_ptr = navigator.GetFeedback();
      auto ptr_spin = std::static_pointer_cast<const nav2_msgs::action::Spin::Feedback>(feedback_ptr);
      std::cout << "Feedback: angular traveled " << ptr_spin->angular_distance_traveled << std::endl;
  }



// class robotScan : public rclcpp::Node{
//     public:
//         robotScan() : Node("robotScan"){
//             publisher_ = this->create_publisher<std_msgs::msg::Float32>("robotScan", 1000);
//             subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&robotScan::processScan, this, std::placeholders::_1));
//         }

//     private:
//         void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
//             // process the scan and find the pillar
//             // find the pillar
//             std::vector<float> ranges = msg->ranges;
//             std::vector<float>::iterator it = std::min_element(ranges.begin(), ranges.end());
//             int index = std::distance(ranges.begin(), it);
//             std::cout << "The pillar is at " << index << std::endl;
//             std_msgs::msg::Float32 msg;
//             msg.data = index;
//             publisher_->publish(msg);
//         }

//         rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
//         rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
// };


 if (ObstacleDetector.HasObstacle()){
        auto obstaclePos = ObstacleDetector.GetObstaclePosition();
        RCLCPP_INFO(node->get_logger(), "Obstacle detected at position (%f, %f)",
            obstaclePos.x, obstaclePos.y);
    }
class ObstacleDetector{
  public:
    ObstacleDetector(double threshold, double min_distance) : 
        threshold_(threshold), min_distance_(min_distance) {}

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg){
        std::vector<geometry_msgs::msg::Point> obstacles;
        double angle = scan_msg->angle_min;
        for(auto range : scan_msg->ranges)
        {
            if(range < min_distance_) {
                geometry_msgs::msg::Point obstacle;
                obstacle.x = range * cos(angle);
                obstacle.y = range * sin(angle);
                obstacle.z = 0;
                obstacles.push_back(obstacle);
            }
            angle += scan_msg->angle_increment;
        }

        if(obstacles.size() > 0) {
            std::cout << "Obstacle detected at:";
            for(auto obstacle : obstacles) {
                std::cout << " (" << obstacle.x << ", " << obstacle.y << ")";
            }
            std::cout << std::endl;
        }
    }

  private:
      double threshold_;
      double min_distance_;
};

bool ObstacleDetector::HasObstacle(){
  for (const auto& range : laser_ranges_) {
    if (range < min_distance_) {
      return true;
    }
  }
  return false;
}


geometry_msgs::msg::Point ObstacleDetector::GetObstaclePosition()
{
  // Loop through each obstacle and find the closest one
  double min_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point closest_obstacle;
  for (const auto& obstacle : obstacles_)
  {
    double distance = std::sqrt(std::pow(obstacle.x, 2) + std::pow(obstacle.y, 2));
    if (distance < min_distance)
    {
      closest_obstacle = obstacle;
      min_distance = distance;
    }
  }

  return closest_obstacle;
}
double arrX[12] = {-1.5, 1.5, 1.75, -2, -2, 1.75, 1.75, 0.5, 0.5, -0.5, -0.5, -2};
  double arrY[12] = {-1.75, -0.5, -0.5, 0.5, 0.5, 1.5, 1.75, -1.75, -1.75, 1.75, -0.5};

  int arraySize = sizeof(arrX) / sizeof(double);
  for (int i = 0; i < arraySize; i++){
    goal->position.x = arrX[i];
    goal->position.y = arrY[i];
    goal->orientation.w = 1;
    // move to new pose
    navigator.GoToPose(goal);
    while (!navigator.IsTaskComplete()){
      // if goal was aborted
      if (navigator.GoToPose(goal) == false){
        // std::cout << "Goal was aborted" << std::endl;
        // break;
      } 
      
    }
      
      
  }
  // amcl pose subscriber
  auto node = std::make_shared<rclcpp::Node>("my_node");

  // Subscribe to the /amcl_pose topic
  auto subscription = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),std::bind(&amclPoseCallback, std::placeholders::_1));

  // rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("amcl_sub");
  // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, amclCallback);
class PoseSubscriber : public rclcpp::Node{
  public:
    PoseSubscriber() : Node("pose_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&PoseSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
    {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      // do something with x and y
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};


std::srand(std::time(nullptr));
double randNum = (double)std::rand() / RAND_MAX; * 2 - 1;
double offsetRandNum = randNum * 0.25;


*/
