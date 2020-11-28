#include <frankx/frankx.hpp>
// using namespace frankx;

int main(int argc, char** argv){
    // Connect to the robot with the FCI IP address
    frankx::Robot robot("172.27.23.65");

    // Reduce velocity and acceleration of the robot
    robot.setDynamicRel(0.05);

    // Move the end-effector 20cm in positive x-direction
    auto motion = frankx::LinearRelativeMotion(frankx::Affine(0.2, 0.0, 0.0));

    // Finally move the robot
    robot.move(motion);
    return 0;
}
