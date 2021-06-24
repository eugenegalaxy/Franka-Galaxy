#include <franka_driver/franka_driver.h>
#include <string>
// #include <iostream>
#include <math.h> // for M_PI

char x;

double convert_rad_to_deg(double rad_value){
  double degree = rad_value * 180 / M_PI;
  // std::cout << std::to_string(rad_value) + " rad is " + std::to_string(degree) + " degrees.\n";
  return degree;
}

std::array<double, 7> read_all_joint_pos(std::string robot_ip, bool return_degrees=false){
  franka::Robot robot(robot_ip);
  std::array<double, 7> curr_joint_pos_rad = robot.readOnce().q;
  std::array<double, 7> curr_joint_pos_deg;

  for(int i=0;i<7;i++){
    curr_joint_pos_deg[i] = curr_joint_pos_rad[i] * 180 / M_PI;
  }

  for(int i=0;i<7;i++){ 
    std::cout << "Joint " + std::to_string(i) + ": " + std::to_string(curr_joint_pos_rad[i]) + " rad";
    std::cout << " (" + std::to_string(curr_joint_pos_deg[i]) + " degrees) \n";
    };
  
  std::cout << "Array to copy (rad):" << std::endl;
  std::cout << "{";
  for(int i=0;i<7;i++){
    if (i < 6){std::cout << std::to_string(curr_joint_pos_rad[i]) + ", ";}
    else{std::cout << std::to_string(curr_joint_pos_rad[i]) + "}\n";};
  }

  std::cout << "Array to copy (degrees):" << std::endl;
  std::cout << "{";
  for(int i=0;i<7;i++){
    if (i < 6){std::cout << std::to_string(curr_joint_pos_deg[i]) + ", ";}
    else{std::cout << std::to_string(curr_joint_pos_deg[i]) + "}\n";};
  }


  if (return_degrees == false){
    return curr_joint_pos_rad;
  }
  else{
    return curr_joint_pos_deg;
  }
}




int main(int argc, char** argv)
{
  try{
    // bool return_degrees = false;
    // auto poses = read_all_joint_pos(robot_ip, return_degrees); 

    // zero_torque_mode()

    // for(int i=0;i<7;i++){std::cout << std::to_string(poses[i]) + " ";};
    // std::cout << "\n";

    // set_vel_acc_jerk(robot_ip, 1);

    std::string robot_ip = "172.27.23.65";

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
