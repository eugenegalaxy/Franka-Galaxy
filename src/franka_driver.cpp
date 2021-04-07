#include <franka_driver/franka_driver.h>

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
void set_default_behavior(std::string robot_ip) {
  franka::Robot robot(robot_ip);
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
 * @param[in] q_goal Target joint positions in radians.
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5.
 *
 */
void move_pos_joint(std::string robot_ip, std::array<double, 7> q_goal, double speed_factor){
  franka::Robot robot(robot_ip);
  robot.automaticErrorRecovery();
  MotionGenerator joint_positions_motion_generator(speed_factor, q_goal);
  robot.control(joint_positions_motion_generator);
  DEBUG2("Robot has been moved. Joint values:", q_goal, 7);
}

/**
 * Moves the robot to "HOME" (also "PRE-GRASP") configuration.
 * @note  o---          Symbols:
 * @note  |   o             =  : workspace table
 * @note  o    \-o.         o  : robot joint
 * @note   \      o      - / | : robot links
 * @note    o     v         v  : end-effector (gripper)
 * @note    |     
 * @note    o  
 * @note ==============
 * 
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5.
 *
 */
void move_pos_home(std::string robot_ip, double speed_factor){
  franka::Robot robot(robot_ip);
  robot.automaticErrorRecovery();
  //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  std::array<double, 7> q_goal = {{-0.1956, -0.2230, -0.0413, -2.4106, -0.0088, 2.1743, 0.5597}};  
  MotionGenerator joint_positions_motion_generator(speed_factor, q_goal);
  robot.control(joint_positions_motion_generator);
  DEBUG("Robot has been moved to 'Home' position.");
}

/**
 * Moves the robot to "TRANSPORTABLE" configuration.
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
 * @warning just an arbitrary compact form.
 * 
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5.
 */
void move_pos_transportable(std::string robot_ip, double speed_factor){
  franka::Robot robot(robot_ip);
  robot.automaticErrorRecovery();
  std::array<double, 7> q_goal = {{-0.02104, 0.1628, -0.4269, -3.0051, 1.0922, 1.5307, -0.7904 }};
  MotionGenerator joint_positions_motion_generator(speed_factor, q_goal);
  robot.control(joint_positions_motion_generator);
  DEBUG("Robot has been moved to 'Transportable' position.");
}

/**
 * Reads full robot state information. 
 * @note Specific states can be extracted using attributes,
 * @note see https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
 * @example robot_state.q -> Measured joint position.
 * @example robot_state.O_T_EE_d -> Last desired end effector pose of motion generation in base frame. 
 * 
 * @return Full robot state information. Note: this can be easily printed with 'std::cout << robot_state << std::endl;'
 */
franka::RobotState read_robot_state(std::string robot_ip){
  franka::Robot robot(robot_ip);
  auto robot_state = robot.readOnce();
  DEBUG(robot_state);
  return robot_state;
}

/**
 * Converts 3x3 Rotation Matrix to Euler angles.
 * 
 * @param[in] rotationMatrix 3x3 rotation matrix in cv::Mat type.
 * @return 3x1 matrix of Euler angles: rx, ry, rz.
 */
cv::Mat _rot_to_euler(const cv::Mat & rotationMatrix){
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
 * @return [x, y, z, rx, ry, rz]. x,y,z are in meters; rx,ry,rz in radians (Euler angles). 
 */
std::array<double, 6> read_ee_pose(std::string robot_ip){
  franka::Robot robot(robot_ip);
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
  eulers = _rot_to_euler(rotationMatrix);

  std::array<double, 6> EE_pose;  // x,y,z, rx, ry, rz in meters
  EE_pose = {robot_state[12], robot_state[13], robot_state[14],
             eulers.at<double>(0), eulers.at<double>(1), eulers.at<double>(2)};

  DEBUG2("End-effector pose:", EE_pose, 6);
  return EE_pose;
}

/**
  * Read Franka robot status:
  * Other (0), Idle (1), Move (2), Guiding (3),
  * Reflex (4), UserStopped (5), AutomaticErrorRecovery (6)
  * 
  * @return string of Franka status.
  */
std::string read_robot_mode(std::string robot_ip){
  franka::Robot robot(robot_ip);
  franka::RobotMode robot_mode = robot.readOnce().robot_mode;
  std::string mode_string;
  switch(robot_mode){
    case franka::RobotMode::kOther:
      mode_string = "Other";
      break;
    case franka::RobotMode::kIdle:
      mode_string = "Idle";
      break;
    case franka::RobotMode::kMove:
      mode_string = "Move";
      break;
    case franka::RobotMode::kGuiding:
      mode_string = "Guiding";
      break;
    case franka::RobotMode::kReflex:
      mode_string = "Reflex";
      break;
    case franka::RobotMode::kUserStopped:
      mode_string = "UserStopped";
      break;
    case franka::RobotMode::kAutomaticErrorRecovery:
      mode_string = "AutomaticErrorRecovery";
      break;
    default:
      mode_string = "ERROR";
      DEBUG("Robot Mode is unknown. Here is a full robot state:");
      auto full_state = robot.readOnce();
      DEBUG(full_state);
  }
  return mode_string;
}

/**
 * A simple controller commanding zero torque for each joint. Gravity is compensated by the robot.
 */ 
void zero_torque_mode(std::string robot_ip){
  franka::Robot robot(robot_ip);
  robot.automaticErrorRecovery();
  robot.control([&](const franka::RobotState&, franka::Duration) -> franka::Torques {
      return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    });
}

/**
 * Performs homing of the gripper. After changing the gripper fingers, a homing needs to be done.
 * This is needed to estimate the maximum grasping width.
 * 
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_homing(std::string robot_ip){
  franka::Gripper gripper(robot_ip);
  bool status = gripper.homing(); // Gripper Calibration
  if(status == true){
    DEBUG("Gripper homed.");
  }
  else{
    DEBUG("Failed to home gripper.");
  }
  return status;
}

/**
 * Moves the gripper fingers to a specified width.
 *
 * @param[in] width Intended opening width. [m] (Max is 0.08)
 * @param[in] speed Closing speed. [m/s] (Max is ~0.2)
 * 
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_move(std::string robot_ip, double width, double speed){
  franka::Gripper gripper(robot_ip);
  bool status = gripper.move(width, speed); // Gripper Calibration
  if(status == true){
    std::string debug_print = "Gripper moved to " + std::to_string(width) + "m with " + std::to_string(speed) + "m/s.";  
    DEBUG(debug_print);
  }
  else{
    DEBUG("Failed to move gripper.");
  }
  return status;
}

/**
 * Grasps an object. An object is considered grasped if the distance 'd' between the gripper fingers satisfies 
 * equation: (width - epsilon_inner) < d < (width - epsilon_outer).
 *
 * @param[in] width Intended opening width. [m]
 * @param[in] speed Closing speed. [m/s]
 * @param[in] force Grasping force. [N]
 * @param[in] epsilon_inner Maximum tolerated deviation when the actual grasped width is SMALLER than the commanded grasp width.
 * @param[in] epsilon_outer Maximum tolerated deviation when the actual grasped width is LARGER than the commanded grasp width.
 * 
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_grasp(std::string robot_ip, double width, double speed, double force, double epsilon_inner, double epsilon_outer){
  franka::Gripper gripper(robot_ip);
  bool status = gripper.grasp(width, speed, force , epsilon_inner, epsilon_outer);
  if(status == true){
    std::string str1 = "Object grasped at " + std::to_string(width) + "m width" +  
                              "with " + std::to_string(speed) + "m/s speed and " + std::to_string(force) + " N force.";
    std::string str2 = "Inner epsilon (tolerance) is " + std::to_string(epsilon_inner) + 
                       "m, outer epsilon is " + std::to_string(epsilon_outer) + "m.";                     
    DEBUG(str1);
    DEBUG(str2);
  }
  else{
    DEBUG("Failed to move grasp the object.");
  }
  return status;
}

/**
 * Stops a currently running gripper move or grasp.
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_stop(std::string robot_ip){
  franka::Gripper gripper(robot_ip);
  bool status = gripper.stop();
  if(status == true){
    DEBUG("Gripper stopped");
  }
  else{
    DEBUG("Failed to stop gripper.");
  }
  return status;
}

/**
 * Reads gripper state information. 
 * @note Specific states can be extracted using attributes,
 * @example robot_state.max_width -> Maximum finger width.
 * 
 * @return Gripper state information. Note: this can be easily printed with 'std::cout << gripper_state << std::endl;'
 */
franka::GripperState read_gripper_state(std::string robot_ip){
  franka::Gripper gripper(robot_ip);
  franka::GripperState gripper_state = gripper.readOnce();
  return gripper_state;
}


//================================ FRANKX functions ==================================

bool move_linear_rel_cartesian(std::string robot_ip, double x, double y, double z){ // in meters
    frankx::Robot robot(robot_ip);
    robot.recoverFromErrors();
    auto motion = frankx::LinearRelativeMotion(frankx::Affine(x, y, z));
    auto status = robot.move(motion);
    return status;
}

void set_vel_acc_jerk(std::string robot_ip, double ratio){ // from 0 (min) to 1 (max)
    frankx::Robot robot(robot_ip);
    robot.recoverFromErrors();
    robot.setDynamicRel(ratio); // Set velocity,acceleration, and jerk to 'ratio' %.
}

void set_velocity(std::string robot_ip, double ratio){
    frankx::Robot robot(robot_ip);
    robot.recoverFromErrors();
    robot.velocity_rel = ratio;
}

void set_acceleration(std::string robot_ip, double ratio){
    frankx::Robot robot(robot_ip);
    robot.recoverFromErrors();
    robot.acceleration_rel = ratio; 
}

void set_jerk(std::string robot_ip, double ratio){
    frankx::Robot robot(robot_ip);
    robot.recoverFromErrors();
    robot.jerk_rel = ratio; 
}
