//Group number 4, Abdelrahman Amira, Victor Fan He,  Zainib Mohammad
//Part 1
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cmath>

using namespace PlayerCc;

// Function headers
player_pose2d_t readPosition(LocalizeProxy &lp)
{
  player_localize_hypoth_t hypothesis;
  player_pose2d_t pose;
  uint32_t hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.
  hCount = lp.GetHypothCount();

  if (hCount > 0)
  {
    hypothesis = lp.GetHypoth(0);
    pose = hypothesis.mean;
  }

  return pose;

}
void printRobotData(BumperProxy &bp, player_pose2d_t pose);

int main(int argc, char *argv[])
{
  // Variables
  double speed;         // How fast do we want the robot to go forwards?
  double turnrate;      // How fast do we want the robot to turn?
  player_pose2d_t pose; // For handling localization data

  // Set up proxies
  PlayerClient robot("localhost");
  BumperProxy bp(&robot, 0);
  Position2dProxy pp(&robot, 0);
  LocalizeProxy lp(&robot, 0);

  // Allow the program to take charge of the motors
  pp.SetMotorEnable(true);

  // Destination coordinates
  double destinationX = 5.0;
  double destinationY = -3.5;
  double distanceThreshold = 0.5; // Stop when within half a meter of the destination

  // Proportional control parameters
  double kp_translation = 0.2; // Proportional gain for translational movement
  double kp_rotation = 0.5;    // Proportional gain for rotational movement

  while (true)
  {
    // Update information from the robot
    robot.Read();

    // Read new information about position
    pose = readPosition(lp);

    // Print data on the robot to the terminal
    printRobotData(bp, pose);

    // Calculate the angle to the destination using atan2
    double angleToDestination = atan2(destinationY - pose.py, destinationX - pose.px);

    // Calculate the difference between the current orientation and the desired direction
    double angleDifference = angleToDestination - pose.pa;

    // Proportional control for rotational movement
    turnrate = kp_rotation * angleDifference;

    // Proportional control for translational movement
    double distanceToDestination = sqrt(pow(destinationX - pose.px, 2) + pow(destinationY - pose.py, 2));
    speed = kp_translation * distanceToDestination;

    // Limit speed to avoid overshooting
    if (speed > 1.0)
    {
      speed = 1.0;
    }

    // Stop if close to the destination
    if (distanceToDestination < distanceThreshold)
    {
      speed = 0;
      turnrate = 0;
      std::cout << "Reached the destination!" << std::endl;
      break; // Exit the control loop
    }

    // Send the commands to the robot
    pp.SetSpeed(speed, turnrate);
  }

  return 0;
}
void printRobotData(BumperProxy &bp, player_pose2d_t pose)
{
  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;
}



