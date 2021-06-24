// #include <franka_driver/franka_driver.h>
// #include <string>

// char x;

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

// int main(int argc, char** argv)
// {
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
//   try{

//     } 
//     catch (const franka::Exception& e){
//         std::cout << e.what() << std::endl;
//         return -1;
//     }
//   return 0;
// }
