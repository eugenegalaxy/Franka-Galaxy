
#include <cmath>
#include <iostream>
#include <limits> // for quiet_NaN
#include <string>

#include <opencv2/opencv.hpp>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include <frankx/frankx.hpp>

#include <franka_plugin/joint_pose_motion_generator.h>



void set_default_behavior(std::string robot_ip);

/**
 * Moves each robot joint to a goal position using Joint Pose controller.
 *
 * @param[in] q_goal Target joint positions in radians.
 * @param[in] speed_factor General speed factor in range [0, 1]. Default value is set at 0.5.
 *
 */
void move_pos_joint(std::string robot_ip, std::array<double, 7> q_goal, double speed_factor=0.5);

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
void move_pos_home(std::string robot_ip, double speed_factor=0.5);

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
void move_pos_transportable(std::string robot_ip, double speed_factor=0.5);

/**
 * Reads full robot state information. 
 * @note Specific states can be extracted using attributes,
 * @note see https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
 * @example robot_state.q -> Measured joint position.
 * @example robot_state.O_T_EE_d -> Last desired end effector pose of motion generation in base frame. 
 * 
 * @return Full robot state information. Note: this can be easily printed with 'std::cout << robot_state << std::endl;'
 */
franka::RobotState read_robot_state(std::string robot_ip);

/**
 * Converts 3x3 Rotation Matrix to Euler angles.
 * 
 * @param[in] rotationMatrix 3x3 rotation matrix in cv::Mat type.
 * @return 3x1 matrix of Euler angles: rx, ry, rz.
 */
cv::Mat _rot_to_euler(const cv::Mat & rotationMatrix);

/**
 * Returns robot end-effector pose in base frame.
 * @return [x, y, z, rx, ry, rz]. x,y,z are in meters; rx,ry,rz in radians (Euler angles). 
 */
std::array<double, 6> read_ee_pose(std::string robot_ip);

/**
  * Read Franka robot status:
  * Other (0), Idle (1), Move (2), Guiding (3),
  * Reflex (4), UserStopped (5), AutomaticErrorRecovery (6)
  * 
  * @return string of Franka status.
  */
std::string read_robot_mode(std::string robot_ip);

/**
 * A simple controller commanding zero torque for each joint. Gravity is compensated by the robot.
 */ 
void zero_torque_mode(std::string robot_ip);

/**
 * Performs homing of the gripper. After changing the gripper fingers, a homing needs to be done.
 * This is needed to estimate the maximum grasping width.
 * 
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_homing(std::string robot_ip);

/**
 * Moves the gripper fingers to a specified width.
 *
 * @param[in] width Intended opening width. [m]
 * @param[in] speed Closing speed. [m/s]
 * 
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_move(std::string robot_ip, double width, double speed);

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
bool gripper_grasp(std::string robot_ip, double width, double speed, double force, 
                   double epsilon_inner = 0.005, double epsilon_outer = 0.005);

/**
 * Stops a currently running gripper move or grasp.
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool gripper_stop(std::string robot_ip);

/**
 * Reads gripper state information. 
 * @note Specific states can be extracted using attributes,
 * @example robot_state.max_width -> Maximum finger width.
 * 
 * @return Gripper state information. Note: this can be easily printed with 'std::cout << gripper_state << std::endl;'
 */
franka::GripperState read_gripper_state(std::string robot_ip);

//================================ FRANKX functions ==================================

/**
 * Moves robot to a RELATIVE position in Cartesian space.
 *
 * @param[in] x x-axis translation in meters.
 * @param[in] y y-axis translation in meters.
 * @param[in] z z-axis translation in meters.
 * 
 * @return status - 'true' is executed successfully, 'false' if not.
 */
bool move_linear_rel_cartesian(std::string robot_ip, double x, double y, double z);

/**
 * Set velocity, acceleration and jerk all at once to Franka robot. From 0 (min) to 1 (MAX) %.
 *
 * @param[in] ratio Percentage of max value [0-1].
 * 
 */
void set_vel_acc_jerk(std::string robot_ip, double ratio=0.2);

/**
 * Set velocity to Franka robot. From 0 (min) to 1 (MAX) %.
 *
 * @param[in] ratio Percentage of max value [0-1].
 * 
 */
void set_velocity(std::string robot_ip, double ratio=0.2);

/**
 * Set acceleration to Franka robot. From 0 (min) to 1 (MAX) %.
 *
 * @param[in] ratio Percentage of max value [0-1].
 * 
 */
void set_acceleration(std::string robot_ip, double ratio=0.1);

/**
 * Set jerk to Franka robot. From 0 (min) to 1 (MAX) %.
 *
 * @param[in] ratio Percentage of max value [0-1].
 * 
 * @note Not sure here what values are valid to apply. TEST IT!
 */
void set_jerk(std::string robot_ip, double ratio=0.01);
