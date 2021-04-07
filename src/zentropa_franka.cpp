#include <franka_driver/franka_driver.h>
#include <string>

#define MAX_GRIPPER_SPEED 0.2

std::string robot_ip = "172.27.23.65";

void gripper_simulate_speaking(std::vector<double> gripper_poses){
    for (auto i : gripper_poses)
        gripper_move(robot_ip, i, MAX_GRIPPER_SPEED);
}

int main(int argc, char** argv)
{ 
    try{
        std::vector<double> example_gripper_poses = {
            0, 0.04, 0.01, 0.06, 0.02, 0.05, 0.01, 0.06, 0.02, 0,06, 0, 0.04, 0.01, 0.06, 0.02, 0.05, 0.01, 0.06, 0.02, 0,06};
        gripper_simulate_speaking(example_gripper_poses);
    }
    catch (const franka::Exception& e){
        std::cout << e.what() << std::endl;
        return -1;
    }
  return 0;
}
