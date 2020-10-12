#include <cmath>
#include <iostream>
#include <limits> // for quiet_NaN

#include <opencv2/opencv.hpp>
#include <franka/exception.h>
#include <franka/robot.h>

#include <franka_plugin/joint_pose_motion_generator.h>

#define ROBOT_IP_STR "172.27.23.65"

#define DEBUG(x) do { \
  if (DEBUGGING_ENABLED) { std::cerr << x << std::endl; } \
} while (0)

#define DEBUG2(msg, x, size_n) do { \
  if (DEBUGGING_ENABLED){  \
    std::cout << msg; std::cout << " "; \
    for(int i=0;i<size_n;i++){ std::cout << x[i]; std::cout << " ";} \
    std::cout << "" << std::endl;} \
} while (0)

bool DEBUGGING_ENABLED = true;

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 * @note Selected values are chosen by guessing, feel free to modify them if needed.
 */
void setDefaultBehavior() {
  franka::Robot robot(ROBOT_IP_STR);
  robot.automaticErrorRecovery();

  // Set the collision behavior. 
  std::array<double, 7> lower_torque_thresholds_acceleration{{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
  std::array<double, 7> upper_torque_thresholds_acceleration{{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 7> lower_torque_thresholds_nominal{{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
  std::array<double, 7> upper_torque_thresholds_nominal{{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};

  std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  robot.setCollisionBehavior(lower_torque_thresholds_acceleration,
                             upper_torque_thresholds_acceleration,
                             lower_torque_thresholds_nominal,
                             upper_torque_thresholds_nominal,
                             lower_force_thresholds_acceleration,
                             upper_force_thresholds_acceleration,
                             lower_force_thresholds_nominal,
                             upper_force_thresholds_nominal);

  std::array<double, 7UL> joint_impedances{{3000, 3000, 3000, 2500, 2500, 2000, 2000}};
  robot.setJointImpedance(joint_impedances);

  std::array<double, 6UL> cartesian_impedances{{3000, 3000, 3000, 300, 300, 300}};
  robot.setCartesianImpedance(cartesian_impedances);
  DEBUG("Default collision behavior is set.");
}

/**
 * Moves each robot joint to a goal position using Joint Pose controller.
 *
 * @param[in] q_goal Target joint positions in radians
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5
 *
 */
void moveJointPos(std::array<double, 7> q_goal, double speed_factor=0.5){
  franka::Robot robot(ROBOT_IP_STR);
  robot.automaticErrorRecovery();
  MotionGenerator joint_positions_motion_generator(speed_factor, q_goal);
  robot.control(joint_positions_motion_generator);
  DEBUG2("Robot has been moved. Joint values:", q_goal, 7);
}

/**
 * Moves the robot to "HOME" (also "PRE-GRASP") configuration  
 * @note  o---          Symbols:
 * @note  |   o             =  : workspace table
 * @note  o    \-o.         o  : robot joint
 * @note   \      o      - / | : robot links
 * @note    o     v         v  : end-effector (gripper)
 * @note    |     
 * @note    o  
 * @note ==============
 * 
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5
 *
 */
void moveHomePos(double speed_factor=0.5){
  franka::Robot robot(ROBOT_IP_STR);
  robot.automaticErrorRecovery();
  //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  std::array<double, 7> q_goal = {{-0.1956, -0.2230, -0.0413, -2.4106, -0.0088, 2.1743, 0.5597}};  
  MotionGenerator joint_positions_motion_generator(speed_factor, q_goal);
  robot.control(joint_positions_motion_generator);
  DEBUG("Robot has been moved to 'Home' position.");
}

/**
 * Moves the robot to "TRANSPORTABLE" configuration  
 * @note     -o-       Symbols:
 * @note    |   |          =  : workspace table
 * @note    o   o          o  : robot joint
 * @note    |   |       - / | : robot links
 * @note    o   |          ^  : end-effector (gripper)
 * @note    |  <o  
 * @note    o    
 * @note ===========
 * 
 * @warning This is not the pose used when packing/unpacking robot from original boxing, but
 * @warning just an arbitrary compact form
 * 
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5
 *
 */
void moveTransportablePos(double speed_factor=0.5){
  franka::Robot robot(ROBOT_IP_STR);
  robot.automaticErrorRecovery();
  std::array<double, 7> q_goal = {{-0.02104, 0.1628, -0.4269, -3.0051, 1.0922, 1.5307, -0.7904 }};
  MotionGenerator joint_positions_motion_generator(speed_factor, q_goal);
  robot.control(joint_positions_motion_generator);
  DEBUG("Robot has been moved to 'Transportable' position.");
}

/**
 * Returns full robot state information. 
 * @note Specific states can be extracted using attributes,
 * @note see https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
 * @example robot_state.q -> Measured joint position.
 * @example robot_state.O_T_EE_d -> Last desired end effector pose of motion generation in base frame. 
 */
franka::RobotState readRobotState(){
  franka::Robot robot(ROBOT_IP_STR);
  auto robot_state = robot.readOnce();
  DEBUG(robot_state);
  return robot_state;
}

/**
 * Converts 3x3 Rotation Matrix to Euler angles
 * 
 * @param[in] rotationMatrix 3x3 rotation matrix in cv::Mat type.
 */
cv::Mat rot2euler(const cv::Mat & rotationMatrix){
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998){ // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998){ // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else{
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }
  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;
  return euler;
}

/**
 * Returns robot end-effector pose in base frame.
 * x,y,z are in meters; rx,ry,rz are in radians (Euler angles). 
 *
 */
std::array<double, 6> readEEpose(){
  franka::Robot robot(ROBOT_IP_STR);
  auto robot_state = robot.readOnce().O_T_EE_d;

  std::array<double, 9> rotation_vector;
  rotation_vector = {robot_state[0],robot_state[1], robot_state[2],
                     robot_state[4],robot_state[5], robot_state[6],
                     robot_state[8],robot_state[9], robot_state[10]};

  cv::Mat rotationMatrix(3, 3, CV_64FC1);
  for(int i = 0; i < 3; i++) 
      for(int j = 0; j < 3; j++) 
          rotationMatrix.at<double>(i,j)= rotation_vector[i+j];

  cv::Mat eulers(3, 1, CV_64F);
  eulers = rot2euler(rotationMatrix);

  std::array<double, 6> EE_pose;  // x,y,z, rx, ry, rz in meters
  EE_pose = {robot_state[12], robot_state[13], robot_state[14],
             eulers.at<double>(0), eulers.at<double>(1), eulers.at<double>(2)};

  DEBUG2("End-effector pose:", EE_pose, 6);
  return EE_pose;
}

int main(int argc, char** argv)
{
  try
  {
    setDefaultBehavior();
    moveHomePos();
    auto pose1 = readEEpose();
    moveTransportablePos();
    auto pose2 = readEEpose();

  } catch (const franka::Exception& e)
    {
    std::cout << e.what() << std::endl;
    return -1;
    }
  return 0;
}
