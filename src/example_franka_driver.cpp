#include <franka_driver/franka_driver.h>
#include <string>

char x;

// actionlib::SimpleActionServer<franka_driver::FrankaAction>* myGripperServer;

// void ActionCB(const franka_driver::FrankaGoalConstPtr& goal)
// {
//     //action callback

//     //interpret goal and switch based on command

//     switch(goal.commandID)
//     {
//       case 0:
//       //home
//       //call 
      
//       franka driver class.function home
//     }

//     //set actionserver succeeded or failed
// }

int main(int argc, char** argv)
{
//   ROS::init();

//   ros::nodehandle n;

//   //instantiate action server object

//   //declare and instantiate ros publisher

//   //spin up thread to always request robot data at fixed frequency

// // OR
//   ros::rate rate();

//   while(ros::ok())
//   {
//     //read robot state
//     //publish robot state

//     rate.sleep();
//   }

//   ros::spin(); //endless loop
  try{
    std::string robot_ip = "172.27.23.65";
    set_vel_acc_jerk(robot_ip, 1);
    while(true){
      gripper_move(robot_ip, 0, 0.2);
      gripper_move(robot_ip, 0.08, 0.2);

    }
    // set_vel_acc_jerk(robot_ip, 0.2);
    // std::cout << "Type any char to home gripper: ";
    // std::cin >> x;
    // gripper_homing(robot_ip);
    // std::cout << "Type any char to grasp with gripper: ";
    // std::cin >> x;
    // gripper_grasp(robot_ip, 0.07, 0.10, 10, 0.02, 0.02);
    // std::cout << "Type any char to read and print gripper state: ";
    // std::cin >> x;
    // auto gr_state = read_gripper_state(robot_ip);
    // std::cout << gr_state << std::endl;
    // std::cout << "Type any char to set Franka 'default' behaviour: ";
    // std::cin >> x;
    // set_default_behavior(robot_ip);
    // std::cout << "Type any char to move Franka to 'HOME' position: ";
    // std::cin >> x;
    // move_pos_home(robot_ip);
    // std::cout << "Type any char to move Franka to a relative position (+10cm on x): ";
    // std::cin >> x;
    // move_linear_rel_cartesian(robot_ip, 0.1, 0, 0);
    // std::cout << "Type any char to read and print robot state: ";
    // std::cin >> x;
    // auto pose1 = read_robot_state(robot_ip);
    // std::cout << pose1 << std::endl;
    // std::cout << "Type any char to move Franka to 'TRANSPORTABLE' position: ";
    // std::cin >> x;
    // move_pos_transportable(robot_ip);
    // std::cout << "Type any char to read Franka robot mode: ";
    // std::cin >> x;
    // std::string status = read_robot_mode(robot_ip);
    // std::cout << status << std::endl;
    } 
    catch (const franka::Exception& e){
        std::cout << e.what() << std::endl;
        return -1;
    }
  return 0;
}
