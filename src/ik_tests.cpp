/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <stdio.h>
#include <kdl/frames_io.hpp>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <cmath> // std::abs
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <mutex>
#include <ncurses.h>

#define CK

using namespace Eigen;
using namespace ur_rtde;

//RTDEControlInterface rtde_control("172.30.10.1");
//RTDEReceiveInterface rtde_receive("172.30.10.1");
 RTDEControlInterface rtde_control("127.0.0.1");
 RTDEReceiveInterface rtde_receive("127.0.0.1");

int _num_samples;
std::string _chain_start, _chain_end, _urdf_param;
double _timeout;
double timestamp = 0;
bool CKDONE = false;
std::mutex mtx;
std::vector<double> eePos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
std::vector<double> start_joint_pos = {0, 0, 0, 0, 0, 0};
 bool modified = false;
std::thread thr;

void moveArm(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param);

void setEEPos(std::vector<double> &p, double ts = 0)
{
  std::lock_guard<std::mutex> guard(mtx);
  eePos = p;
  modified = true;

  timestamp = ts;
}

static void loopCK()
{

  // std::vector<double> startPose = {-0.1,0.4,0.5, 2.938,-2.82,1.479};
  // std::vector<double> startPose = {-0.1, 0.4, 0.5, 4.356, -0.529, -0.587};
  // rtde_control.servoJ(rtde_control.getInverseKinematics(startPose), 1, 1, 0.1, 0.2, 300);

  // printf("DEBUG:  loopCK\n");
  // printf("DEBUG: eePos.size(): %lu \n", eePos.size());

  while (!CKDONE)
  {
    std::vector<double> pos;
    {
      std::lock_guard<std::mutex> guard(mtx);
      pos = eePos;
    }
    if (eePos.size() == 0 || modified == false)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    else
    {
      std::lock_guard<std::mutex> guard(mtx);
      if (modified)
      {
        // printf("DEBUG: modified\n");
        modified = false;
      }
    }
    // printf("DEBUG: pre moveArm\n");
    // printf("DEBUG: eePos.size(): %lu \n", eePos.size());
    moveArm(_num_samples, _chain_start, _chain_end, _timeout, _urdf_param);
  }
}

void startCK()
{
  thr = std::thread(loopCK);
}

void writeToCsv(std::vector<KDL::JntArray> &vec)
{
  std::ofstream file;
  file.open("/home/cpn/catkin_ws/src/trac_ik_examples/trac_ik_examples/solutions.csv");

  for (int i = 0; i < vec.size(); i++)
  {
    for (unsigned int j = 0; j < 6; j++)
    {
      file << vec.at(i)(j);
      if (j < 5)
      {
        file << ",";
      }
    }
    file << "\n";
  }

  file.close();
  return;
}

template <typename T>
std::vector<double> vecSubtract(const std::vector<T> &vec1, const std::vector<T> &vec2)
{
  std::vector<double> result(vec1.size(), 0.0);
  for (int i = 0; i < vec1.size(); ++i)
  {
    result.at(i) = vec1.at(i) - vec2.at(i);
  }
  return result;
}

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

/* calculates new joint speed based on max velocity and remaining angle difference.
If no joint has a distance more than 3.14 the joint speed equals the difference from actual to goal (per joint) multiplied by a constant*/
std::vector<double> newJointSpeed(std::vector<double> joint_config, std::vector<double> actual_q, std::vector<double> joint_speed, double max_vel)
{
  // printf("DEBUG: newJointSpeed\n");
  std::vector<double> tmp_speed = vecSubtract(joint_config, actual_q);
  std::vector<double> abs_tmp_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < tmp_speed.size() - 1; i++)
  {
    abs_tmp_speed[i] = std::abs(tmp_speed[i]);
  }
  for (int i = 0; i < joint_config.size(); i++)
  {
    if (std::abs(tmp_speed[i]) > 0.2)
    {
      if(tmp_speed[i] > 0)
        joint_speed[i] = max_vel;
      else
        joint_speed[i] = -max_vel;
    }
    else if (std::abs(tmp_speed[i]) > 0.001)
    {
      joint_speed[i] = std::min(tmp_speed[i] * 5, max_vel);
      if (*max_element(abs_tmp_speed.begin(), abs_tmp_speed.end()) > 3.14)
      {
        joint_speed[i] = tmp_speed[i] * (max_vel / *max_element(abs_tmp_speed.begin(), abs_tmp_speed.end()));
      }
    }
    else
    {
      joint_speed[i] = 0.0;
    }
  }

  return joint_speed;
}

int findClosestSolution(std::vector<double> actual_q, std::vector<KDL::JntArray> solutions)
{
  printf("findClosestSolution\n");
  int index_min = 0;
  double min = 999.0;
  double difference = 0.0;
  for (int i = 0; i < solutions.size(); i++)
  {
    difference = 0.0;
    for (int j = 0; j < actual_q.size() - 1; j++)
    {
      difference += std::abs(solutions.at(i)(j) - actual_q.at(j));
    }
    if (difference < min)
    {
      min = difference;
      index_min = i;
    }
  }
  std::cout << (index_min) << std::endl;
  printf("\n");
  return index_min;
}

void testRandomSamples(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  printf("Testing random samples:\n");
  double eps = 1e-5;
  double total_time = 0;
  uint success = 0;
  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; // lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);

  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  std::cout << "Using" << chain.getNrOfJoints() << "joints" << std::endl;

  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  for (uint i = 0; i < num_samples; i++)
  {
    for (uint j = 0; j < ll.data.size(); j++)
    {
      q(j) = fRand(ll(j), ul(j));
    }
    JointList.push_back(q);
  }
  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;

  std::cout << "*** Testing TRAC-IK with " << num_samples << " random samples" << std::endl;

  std::vector<KDL::JntArray> solutions;

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time += elapsed;
    if (rc >= 0)
    {
      success++;
      int zeros_found = 0;
      for (unsigned int j = 0; j < 6; j++)
      {
        if (result(j) == 0.0)
        {
          zeros_found += 1;
        }
      }
      if (zeros_found >= 6)
      {
        /*for(unsigned int j=0;j<6;j++){
          std::cout << result(j);
          if(j < 5 ){
             std::cout << ", ";
          }
        }*/
        std::cout << rc << std::endl;
        printf("\n");
      }
      solutions.push_back(result);
    }
    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  writeToCsv(solutions);
  std::cout << "TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample" << std::endl;
}

KDL::JntArray calculateSolution(double num_samples, std::vector<double> startpos, std::vector<double> ee_pose, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  // printf("DEBUG: calculateSolution\n");
  KDL::JntArray result;
  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Distance);
  // printf("DEBUG: tracik_solver init done\n");

  KDL::Chain chain;
  KDL::JntArray ll, ul; // lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);

  if (!valid)
  {
    printf("There was no valid KDL chain found");
    return result;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    printf("There were no valid KDL joint limits found");
    return result;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  printf("\nUsing %d joints\n", chain.getNrOfJoints());

  // Set up KDL IK

  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver

  // KDL::ChainIkSolverVel_pinv vik_solver(chain);                                         // PseudoInverse vel solver
  // KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  //  1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::Frame end_effector_pose;
  int rc;

  double total_time = 0;
  uint success = 0;

  std::cout << "*** TRAC-IK with: *** \n"
            << std::endl;
  std::cout << "Start\n"
            << std::endl;
  for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
  {
    std::cout << startpos[i] << std::endl;
  }

  printf("\n");
  KDL::Rotation rotation = KDL::Rotation(ee_pose[0], ee_pose[4], ee_pose[8], ee_pose[1], ee_pose[5], ee_pose[9], ee_pose[2], ee_pose[6], ee_pose[10]);
  // KDL::Rotation rotation = KDL::Rotation(ee_pose[0], ee_pose[1], ee_pose[2], ee_pose[4], ee_pose[5], ee_pose[6], ee_pose[8], ee_pose[9], ee_pose[10]);
  KDL::Vector goal = KDL::Vector(ee_pose[3], ee_pose[7], ee_pose[11]);
  std::cout << "Ration Matrix: " << rotation << "\n"
            << std::endl;
  std::cout << "Goal (xyz): " << goal << "\n"
            << std::endl;

  end_effector_pose = KDL::Frame(rotation, goal);

  KDL::JntArray jnt;
  Eigen::VectorXd temp(6);
  KDL::JntArray jnt_goal(6);
  Eigen::VectorXd temp_goal(6);

  temp(0) = startpos[0];
  temp(1) = startpos[1];
  temp(2) = startpos[2];
  temp(3) = startpos[3];
  temp(4) = startpos[4];
  temp(5) = startpos[5];

  std::vector<double> goalpos = {1.67858, -1.49047, -1.38178, 0.07177, -0.0568994, 0.0680332};
  temp_goal(0) = goalpos[0];
  temp_goal(1) = goalpos[1];
  temp_goal(2) = goalpos[2];
  temp_goal(3) = goalpos[3];
  temp_goal(4) = goalpos[4];
  temp_goal(5) = goalpos[5];

  jnt.data = temp;
  jnt.resize(6);

  jnt_goal.data = temp_goal;
  jnt_goal.resize(6);

  // fk_solver.JntToCart(jnt_goal, end_effector_pose);
  // printf("DEBUG: end_effector_pose\n");
  // std::cout << end_effector_pose << std::endl;
  rc = 0;

  /*printf("DEBUG: jnt\n");
  for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
  {
    std::cout << jnt(i) << std::endl;
  }*/

  double elapsed = 0;
  start_time = boost::posix_time::microsec_clock::local_time();
  // printf("DEBUG: chain number of segments: %u\n", chain.getNrOfSegments());
  rc = tracik_solver.CartToJnt(jnt, end_effector_pose, result);
  diff = boost::posix_time::microsec_clock::local_time() - start_time;
  elapsed = diff.total_nanoseconds() / 1e9;
  std::cout << "Time needed for calculation:" << elapsed << "seconds" << std::endl;

  if (rc < 0)
  {
    printf("rc\n");
    // std:cout << rc << std::endl;
    printf("%i\n", rc);
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
    {
      std::cout << result(i) << std::endl;
    }
    printf("Error: Killing Program!\n");
    exit(-1);
  }
  else
  {
    printf("rc (no error):%i\n", rc);
    printf("Solution:\n");
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
    {
      std::cout << result(i) << std::endl;
    }
    std::vector<KDL::JntArray> solutions;
    tracik_solver.getSolutions(solutions);
    std::cout << "Number of found solutions: " << solutions.size() << std::endl;
    // std::vector<double> goal = {result(0), result(1), result(2), result(3), result(4), result(5)};
    // rtde_control.moveJ(goal);
    printf("\n");
    double lower_table_collision = -2.9670597284;
    double upper_table_collision = -0.16720254234;
    while (result(1) < lower_table_collision || result(1) > upper_table_collision) // -167° || -15°
    {
      printf("Found solution could result in collision with table. Searching for different solution.");
      solutions.erase(std::remove(solutions.begin(), solutions.end(), result), solutions.end());
      if (solutions.size() <= 0)
      {
        printf("Error: Solutions size =0 after remove one or more solutions with collisions. Killing Program!\n");
        exit(-1);
      }
      result = solutions.at(findClosestSolution(rtde_receive.getActualQ(), solutions));
      printf("new Solution:\n");
      for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
      {
        std::cout << result(i) << std::endl;
      }
      printf("\n");
    }
  }

  printf("Pose:\n %f, ", rtde_receive.getActualQ().at(0));
  printf("%f, ", rtde_receive.getActualQ().at(1));
  printf("%f, ", rtde_receive.getActualQ().at(2));
  printf("%f, ", rtde_receive.getActualQ().at(3));
  printf("%f, ", rtde_receive.getActualQ().at(4));
  printf("%f\n", rtde_receive.getActualQ().at(5));
  printf("\n");
  printf("TCP offset x: %f\n", rtde_receive.getActualTCPPose().at(0));
  printf("TCP offset y: %f\n", rtde_receive.getActualTCPPose().at(1));
  printf("TCP offset z: %f\n", rtde_receive.getActualTCPPose().at(2));
  printf("TCP offset rx: %f\n", rtde_receive.getActualTCPPose().at(3));
  printf("TCP offset ry: %f\n", rtde_receive.getActualTCPPose().at(4));
  printf("TCP offset rz: %f\n", rtde_receive.getActualTCPPose().at(5));

  return result;
}

/* move the arm with ur_rtde to the best found solution*/
void moveArm(double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  // printf("DEBUG: moveArm\n");
  std::vector<double> actual_q = rtde_receive.getActualQ();
  //start_joint_pos = actual_q;
  KDL::JntArray result = calculateSolution(num_samples, actual_q, eePos, chain_start, chain_end, timeout, urdf_param);

  printf("\n");

  bool postest = false; // used for testing and getting ee_pose from joint_pos
  if (postest)
  {
    return;
  }
  double sT = 0.002; // sample Time

  std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> goal = {result(0), result(1), result(2), result(3), result(4), result(5)};

  double max_vel = 1.5; // max possible value is 3.14
  double acceleration = 10.0;
  double dt = 0.02;
  actual_q = rtde_receive.getActualQ();

  printf("current joint angles:\n");
  for (int i = 0; i <= actual_q.size() - 1; i++)
    std::cout << actual_q[i] << " " << std::endl;
  printf("\n");
  joint_speed = newJointSpeed(goal, actual_q, joint_speed, max_vel);

  bool is_all_zero = true;
  for (int i = 0; i < joint_speed.size(); i++)
  {
    if (joint_speed[i] != 0.0)
    {
      is_all_zero = false;
      break;
    }
  }

  // move arm until the goal position is reached
  while (is_all_zero == false)
  {
    auto t_start = std::chrono::high_resolution_clock::now();

    rtde_control.speedJ(joint_speed, acceleration, dt);

    auto t_stop = std::chrono::high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < sT)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(sT - t_duration.count()));
    }

    actual_q = rtde_receive.getActualQ();
    joint_speed = newJointSpeed(goal, actual_q, joint_speed, max_vel);

    is_all_zero = true;
    for (int i = 0; i < joint_speed.size(); i++)
    {
      if (joint_speed[i] != 0.0)
      {
        is_all_zero = false;
        break;
      }
    }
    if (modified)
    {
      actual_q = rtde_receive.getActualQ();

      printf("reached joint angles:\n");
      for (int i = 0; i <= actual_q.size() - 1; i++)
        std::cout << actual_q[i] << " " << std::endl;
      printf("\n");
      // compare acutal TCP Pose with wanted ee_pose
      std::cout << "compare reached position with wanted position:\n"
                << std::endl;
      std::cout << "TCP x:" << rtde_receive.getActualTCPPose().at(0) - eePos.at(3) << "\n"
                << std::endl;
      std::cout << "TCP y:" << rtde_receive.getActualTCPPose().at(1) - eePos.at(7) << "\n"
                << std::endl;
      std::cout << "TCP z:" << rtde_receive.getActualTCPPose().at(2) - eePos.at(11) << "\n"
                << std::endl;
      return;
    }
  }

  actual_q = rtde_receive.getActualQ();

  printf("reached joint angles:\n");
  for (int i = 0; i <= actual_q.size() - 1; i++)
    std::cout << actual_q[i] << " " << std::endl;
  printf("\n");
  // compare acutal TCP Pose with wanted ee_pose
  std::cout << "compare reached position with wanted position:\n"
            << std::endl;
  std::cout << "TCP x:" << rtde_receive.getActualTCPPose().at(0) - eePos.at(3) << "\n"
            << std::endl;
  std::cout << "TCP y:" << rtde_receive.getActualTCPPose().at(1) - eePos.at(7) << "\n"
            << std::endl;
  std::cout << "TCP z:" << rtde_receive.getActualTCPPose().at(2) - eePos.at(11) << "\n"
            << std::endl;
  rtde_control.speedStop();

  printf("End Move Arm \n");
}

void tests()
{
  double temp_z = 0.274;

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;
  start_time = boost::posix_time::microsec_clock::local_time();

  std::vector<double> hin_ee_pose = {-0.9993266, 0.0308951, -0.0197921, -0.15177,
                                     -0.0157949, 0.1246475, 0.9920754, 0.4,
                                     0.0331173, 0.9917200, -0.1240756, 0.736058};

  std::vector<double> her_ee_pose = {-0.9993266, 0.0308951, -0.0197921, -0.15177,
                                     -0.0157949, 0.1246475, 0.9920754, 0.4,
                                     0.0331173, 0.9917200, -0.1240756, 0.274};
  setEEPos(her_ee_pose);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (true)
  {
    // test for fluid movement
    if (temp_z < 0.244)
    {
      temp_z += 0.05;
    }
    /*if (rtde_receive.getActualTCPPose().at(2) > 0.73)
      break;
    */
    std::vector<double> new_ee_pose = {-0.9993266, 0.0308951, -0.0197921, -0.114,
                                       -0.0157949, 0.1246475, 0.9920754, 0.502,
                                       0.0331173, 0.9917200, -0.1240756, temp_z};
    setEEPos(new_ee_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    /*setEEPos(hin_ee_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (std::abs(rtde_receive.getActualTCPPose().at(0) - hin_ee_pose.at(3)) < 0.01)
    {
      if (std::abs(rtde_receive.getActualTCPPose().at(1) - hin_ee_pose.at(7)) < 0.01)
      {
        if (std::abs(rtde_receive.getActualTCPPose().at(2) - hin_ee_pose.at(11)) < 0.01)
        {
          std::cout << "hin_ee_pose has been reached! Killing programm!" << std::endl;
          break;
        }
      }
    }

    setEEPos(her_ee_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (std::abs(rtde_receive.getActualTCPPose().at(0) - her_ee_pose.at(3)) < 0.01)
    {
      if (std::abs(rtde_receive.getActualTCPPose().at(1) - her_ee_pose.at(7)) < 0.01)
      {
        if (std::abs(rtde_receive.getActualTCPPose().at(2) - her_ee_pose.at(11)) < 0.01)
        {
          std::cout << "hin_ee_pose has been reached! Killing programm!" << std::endl;
          break;
        }
      }
    }*/

    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    double elapsed = diff.total_nanoseconds() / 1e9;
    double kill_after = 15.0;
    if (elapsed > kill_after)
    {
      std::cout << "Killing program after " << kill_after << " seconds!" << std::endl;
      break;
    }
  }
  return;
}

MatrixXd S(MatrixXd n)
{
  MatrixXd Sn(3, 3);
  Sn << 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0;
  Sn(1) = n(2) * -1;
  Sn(2) = n(1);
  Sn(3) = n(2);
  Sn(5) = n(0) * -1;
  Sn(6) = n(1) * -1;
  Sn(7) = n(0);
  return Sn;
}

// rodrigues calculation for rotation vector -> rotation matrix
std::vector<double> rodrigues(std::vector<double> &_r)
{
  VectorXd r(3);
  r << _r.at(0), _r.at(1), _r.at(2);

  double norm = r.norm();
  double theta = norm;
  // std::vector<double> R(9, 0.0);
  MatrixXd R(3, 3);
  R << 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0;
  if (theta > 1e-30)
  {
    printf("theta > 1e-30");
    MatrixXd n(3, 3);
    n = r / theta; // DivideVectorByValue(r, theta);
    std::cout << "n = " << n << "\n";
    MatrixXd Sn(3, 3);
    Sn = S(n);
    std::cout << "Sn = " << Sn << "\n";
    MatrixXd eye(3, 3);
    eye << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;

    R = (eye + (Sn * sin(theta)));
    std::cout << "R = " << R << "\n";
    std::cout << "Sn.dot(Sn) = " << Sn * Sn << "\n";
    std::cout << "(Sn.dot(Sn) * (1 - cos(theta))) = " << (Sn * Sn) * (1 - cos(theta)) << "\n";
    R = R + ((Sn * Sn) * (1 - cos(theta)));
  }
  else
  {
    MatrixXd Sr(3, 3);
    Sr = S(r);
    double theta2 = theta * theta;
    MatrixXd eye(3, 3);
    eye << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    R = (eye.array() + (1 - sin(theta2) / 6.0));
    R = R * Sr;
    R = R + (0.5 - std::cos(theta2) / 24.0) * (Sr * Sr);
  }
  std::vector<double> Rd(9, 0.0);
  int xy = 0;
  for (int x = 0; x < R.rows(); ++x)
  {
    for (int y = 0; y < R.cols(); y++)
    {
      Rd.at(xy) = R(y, x);
      xy++;
    }
  }
  return Rd;
}

int main(int argc, char **argv)
{
  ROS_INFO("ROS_INFO"); // need to call ROS_INFO one time, so std::cout prints work
  std::cout.flush();
  // printf("DEBUG: main() #1\n");
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  nh.param("num_samples", _num_samples, 1000);
  nh.param("chain_start", _chain_start, std::string(""));
  nh.param("chain_end", _chain_end, std::string(""));

  if (_chain_start == "" || _chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  nh.param("timeout", _timeout, 0.05);
  nh.param("urdf_param", _urdf_param, std::string("/robot_description"));

  if (_num_samples < 1)
    _num_samples = 1;

  // testRandomSamples(num_samples, chain_start, chain_end, timeout, urdf_param);
  // moveArm("172.30.10.1", ee_pose, num_samples, chain_start, chain_end, timeout, urdf_param);

  // std::vector<double> test_start = {1.67858,-1.49047,-1.38178,0.07177,-0.0568994,0.0680332};
  // rtde_control.moveJ(test_start);

  printf("Starting rodrigues test:\n");
  std::vector<double> test = {0.018, -2.168, -2.277};
  std::vector<double> rod_test(9);
  rod_test = rodrigues(test);
  printf("Test: \n");
  for (int j = 0; j < rod_test.size(); j++)
  {
    std::cout << rod_test.at(j) << " ";
  }
  printf("\n");

  std::vector<double> ee_pose = {-0.9993266, 0.0308951, -0.0197921, -0.15177,
                                 -0.0157949, 0.1246475, 0.9920754, 0.4,
                                 0.0331173, 0.9917200, -0.1240756, 0.274};

  std::vector<double> ee_pose_with_rodriguez = {rod_test.at(0), rod_test.at(3), rod_test.at(6), -0.085,
                                                rod_test.at(1), rod_test.at(4), rod_test.at(7), 0.560,
                                                rod_test.at(2), rod_test.at(5), rod_test.at(8), 0.530};

  // printf("DEBUG: main() #2\n");
  setEEPos(ee_pose_with_rodriguez);
  startCK();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // while (true)
  //{
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  tests();

  rtde_control.stopScript();
  rtde_receive.disconnect();

  return 0;
}

/*
Error codes für rc (tracik_solver.CartToJnt):

1 = E_DEGRADED
Converged but degraded solution (e.g. WDLS with psuedo-inverse singular)

0 = E_NOERROR
No error.

-1 = E_NO_CONVERGE
Failed to converge.

-2 = E_UNDEFINED
Undefined value (e.g. computed a NAN, or tan(90 degrees) )

-3 = E_NOT_UP_TO_DATE
Chain size changed.

-4 = E_SIZE_MISMATCH
Input size does not match internal state.

-5 = E_MAX_ITERATIONS_EXCEEDED
Maximum number of iterations exceeded.

-6 = E_OUT_OF_RANGE
Requested index out of range.

-7 = E_NOT_IMPLEMENTED
Not yet implemented.

-8 = E_SVD_FAILED
Internal svd calculation failed.
*/
