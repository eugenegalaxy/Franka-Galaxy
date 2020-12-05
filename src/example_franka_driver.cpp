#include <franka_driver/franka_driver.h>
#include <string>

int main(int argc, char** argv)
{
  try{
  
    std::string robot_ip = "172.27.23.65";
    // set_vel_acc_jerk(robot_ip, 0.2);
    // gripper_homing(robot_ip);
    // gripper_grasp(0.07, 0.10, 10, 0.02, 0.02);
    // auto gr_state = read_gripper_state();
    // std::cout << gr_state << std::endl;
    // set_default_behavior(robot_ip);
    // move_pos_home();
    // move_linear_rel_cartesian(0.1, 0, 0); 
    // auto pose1 = read_robot_state();
    // move_pos_transportable();
    // auto pose2 = read_robot_state();
    // std::string status = read_robot_mode();
    } 
    catch (const franka::Exception& e){
        std::cout << e.what() << std::endl;
        return -1;
    }
  return 0;
}
