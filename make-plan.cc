// Group number 4, Abdelrahman Amira, Victor Fan He, Zainib Mohammad
/**
 * make-plan.cc
 * 
 * Sample code for a robot that has two front bumpers and a laser, and
 * which is provided with localization data.
 *
 * The code also allows the controller to read and write a "plan", a sequence
 * of location that the robot should move to and to read in a "map", a matrix
 * of 1 and 0 values that can be used as an occupancy grid.
 *
 * Written by: Simon Parsons
 * Date:       4th December 2011
 *  
 **/


#include <iostream>
#include <fstream>
#include <vector>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

const int SIZE = 32;

player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);
void readMap(int map[SIZE][SIZE]);
void writeMapWithWaypoints(int map[SIZE][SIZE], const std::vector<player_pose2d_t>& plan);
void printMap(int map[SIZE][SIZE]);
int readPlanLength(void);
bool readPlan(const std::string& filename, std::vector<player_pose2d_t>& plan);
void printPlan(const std::vector<player_pose2d_t>& plan);
void writePlan(const std::vector<player_pose2d_t>& plan);
void generateWavefrontPlan(int oGrid[SIZE][SIZE], std::vector<player_pose2d_t>& plan);

int main(int argc, char *argv[])
{
    int counter = 0;
    double speed;
    double turnrate;
    player_pose2d_t pose;
    int oGrid[SIZE][SIZE];
    std::vector<player_pose2d_t> plan;

    PlayerClient robot("localhost");
    BumperProxy bp(&robot, 0);
    Position2dProxy pp(&robot, 0);
    LocalizeProxy lp(&robot, 0);
    LaserProxy sp(&robot, 0);

    pp.SetMotorEnable(true);

    readMap(oGrid);
    printMap(oGrid);
    writeMapWithWaypoints(oGrid, plan);

    if (!readPlan("plan.txt", plan))
    {
        std::cerr << "Error reading plan from file." << std::endl;
        return 1;
    }

    printPlan(plan);
    writePlan(plan);

    double kp_translation = 0.2;
    double kp_rotation = 0.5;
    double obstacle_distance_threshold = 0.5;

    for (size_t i = 0; i < plan.size(); ++i)
    {
        const player_pose2d_t& waypoint = plan[i];

        while (true)
        {
            robot.Read();
            pose = readPosition(lp);
            printRobotData(bp, pose);

            if (counter > 2)
            {
                printLaserData(sp);
            }

            bool obstacleDetected = false;
            for (size_t j = 0; j < sp.GetCount(); ++j)
            {
                double range = sp.GetRange(j);
                if (range < obstacle_distance_threshold)
                {
                    obstacleDetected = true;
                    break;
                }
            }

            if (obstacleDetected)
            {
                speed = 0;
                turnrate = 0.5;
            }
            else
            {
                double angleToWaypoint = atan2(waypoint.py - pose.py, waypoint.px - pose.px);
                double angleDifference = angleToWaypoint - pose.pa;
                turnrate = kp_rotation * angleDifference;

                double distanceToWaypoint = sqrt(pow(waypoint.px - pose.px, 2) + pow(waypoint.py - pose.py, 2));
                speed = kp_translation * distanceToWaypoint;

                if (speed > 1.0)
                {
                    speed = 1.0;
                }

                if (distanceToWaypoint < 0.5)
                {
                    speed = 0;
                    turnrate = 0;
                    std::cout << "Reached the waypoint!" << std::endl;
                    break;
                }
            }

            std::cout << "Speed: " << speed << std::endl;
            std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

            pp.SetSpeed(speed, turnrate);
            counter++;
        }
    }

    // Write the map with waypoints to map-out.txt
    writeMapWithWaypoints(oGrid, plan);

    return 0;
}

void readMap(int map[SIZE][SIZE])
{
    std::ifstream mapFile;
    mapFile.open("map.txt");

    for (int i = SIZE - 1; i >= 0; i--)
    {
        for (int j = 0; j < SIZE; j++)
        {
            mapFile >> map[i][j];
        }
    }

    mapFile.close();
}

void printMap(int map[SIZE][SIZE])
{
    for (int i = SIZE - 1; i >= 0; i--)
    {
        for (int j = 0; j < SIZE; j++)
        {
            std::cout << map[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{
    std::cout << "Left  bumper: " << bp[0] << std::endl;
    std::cout << "Right bumper: " << bp[1] << std::endl;

    std::cout << "We are at" << std::endl;
    std::cout << "X: " << pose.px << std::endl;
    std::cout << "Y: " << pose.py << std::endl;
    std::cout << "A: " << pose.pa << std::endl;
}

player_pose2d_t readPosition(LocalizeProxy& lp)
{
    player_localize_hypoth_t hypothesis;
    player_pose2d_t pose;
    uint32_t hCount;

    hCount = lp.GetHypothCount();

    if (hCount > 0)
    {
        hypothesis = lp.GetHypoth(0);
        pose = hypothesis.mean;
    }

    return pose;
}

void printLaserData(LaserProxy& sp)
{
    double maxRange, minLeft, minRight, range, bearing;
    int points;

    maxRange = sp.GetMaxRange();
    minLeft = sp.MinLeft();
    minRight = sp.MinRight();
    range = sp.GetRange(5);
    bearing = sp.GetBearing(5);
    points = sp.GetCount();

    std::cout << "Laser says..." << std::endl;
    std::cout << "Maximum distance I can see: " << maxRange << std::endl;
    std::cout << "Number of readings I return: " << points << std::endl;
    std::cout << "Closest thing on left: " << minLeft << std::endl;
    std::cout << "Closest thing on right: " << minRight << std::endl;
    std::cout << "Range of a single point: " << range << std::endl;
    std::cout << "Bearing of a single point: " << bearing << std::endl;
}

int readPlanLength(void)
{
    int length;

    std::ifstream planFile;
    planFile.open("plan.txt");

    planFile >> length;
    planFile.close();

    if ((length % 2) != 0)
    {
        std::cout << "The plan has mismatched x and y coordinates" << std::endl;
        exit(1);
    }

    return length;
}

bool readPlan(const std::string& filename, std::vector<player_pose2d_t>& plan)
{
    std::ifstream file(filename.c_str());

    if (!file.is_open())
    {
        return false;
    }

    int n;
    file >> n;

    if (n <= 0 || n % 2 != 0)
    {
        return false;
    }

    plan.resize(n / 2);

    for (int i = 0; i < n / 2; ++i)
    {
        file >> plan[i].px >> plan[i].py;
    }

    file.close();
    return true;
}

void printPlan(const std::vector<player_pose2d_t>& plan)
{
    std::cout << std::endl;
    std::cout << "   x     y" << std::endl;
    for (size_t i = 0; i < plan.size(); ++i)
    {
        std::cout.width(5);
        std::cout << plan[i].px << " ";
        std::cout.width(5);
        std::cout << plan[i].py << " ";
        if ((i > 0) && ((i % 2) != 0))
        {
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
}

void writePlan(const std::vector<player_pose2d_t>& plan)
{
    std::ofstream planFile;
    planFile.open("plan-out.txt");

    planFile << plan.size() * 2 << " ";
    for (size_t i = 0; i < plan.size(); ++i)
    {
        planFile << plan[i].px << " " << plan[i].py << " ";
    }

    planFile.close();
}

void generateWavefrontPlan(int oGrid[SIZE][SIZE], std::vector<player_pose2d_t>& plan)
{
    // Implementation of wavefront planner
    // Assuming SIZE is the same for both rows and columns
    const int rows = SIZE;
    const int cols = SIZE;

    // Define directional offsets for 8-connected grid
    const int dx[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

    // Create a wavefront grid
    int wavefrontGrid[rows][cols];
    memset(wavefrontGrid, 0, sizeof(wavefrontGrid));

    // Define the start and goal positions
    const int startX = 0;
    const int startY = 0;
    const int goalX = SIZE - 1;
    const int goalY = SIZE - 1;

    // Initialize the wavefront grid with obstacles
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if (oGrid[i][j] == 1)
            {
                wavefrontGrid[i][j] = -1; // Mark obstacles with -1
            }
        }
    }

    // Initialize the wavefront grid with the goal
    wavefrontGrid[goalX][goalY] = 1;

    // Propagate the wavefront until it reaches the start
    int wavefrontValue = 1;
    while (wavefrontGrid[startX][startY] == 0)
    {
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                if (wavefrontGrid[i][j] == wavefrontValue)
                {
                    // Propagate to neighbors
                    for (int k = 0; k < 8; ++k)
                    {
                        int nx = i + dx[k];
                        int ny = j + dy[k];

                        if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && wavefrontGrid[nx][ny] == 0)
                        {
                            wavefrontGrid[nx][ny] = wavefrontValue + 1;
                        }
                    }
                }
            }
        }

        ++wavefrontValue;
    }

    // Extract the plan from the wavefront
    int currentX = startX;
    int currentY = startY;

    while (currentX != goalX || currentY != goalY)
    {
        // Add the current position to the plan
        player_pose2d_t waypoint;
        waypoint.px = currentX;
        waypoint.py = currentY;
        plan.push_back(waypoint);

        // Find the neighbor with the lowest wavefront value
        int minWavefront = wavefrontGrid[currentX][currentY];
        int nextX = currentX;
        int nextY = currentY;

        for (int k = 0; k < 8; ++k)
        {
            int nx = currentX + dx[k];
            int ny = currentY + dy[k];

            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && wavefrontGrid[nx][ny] > 0 &&
                wavefrontGrid[nx][ny] < minWavefront)
            {
                minWavefront = wavefrontGrid[nx][ny];
                nextX = nx;
                nextY = ny;
            }
        }

        // Move to the next position
        currentX = nextX;
        currentY = nextY;
    }

    // Add the goal to the plan
    player_pose2d_t goal;
    goal.px = goalX;
    goal.py = goalY;
    plan.push_back(goal);
}

void writeMapWithWaypoints(int map[SIZE][SIZE], const std::vector<player_pose2d_t>& plan)
{
    std::ofstream mapFile;
    mapFile.open("map-out.txt");

    for (int i = SIZE - 1; i >= 0; i--)
    {
        for (int j = 0; j < SIZE; j++)
        {
            if (map[i][j] == 2)
            {
                mapFile << 2 << " ";
            }
            else
            {
                mapFile << map[i][j] << " ";
            }
        }
        mapFile << std::endl;
    }

    for (size_t i = 0; i < plan.size(); ++i)
    {
        int x = static_cast<int>(plan[i].px) + SIZE / 2;
        int y = static_cast<int>(plan[i].py) + SIZE / 2;
        mapFile << x << " " << y << " ";
    }

    mapFile.close();
}
