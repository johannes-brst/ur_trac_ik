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

#define CK

using namespace ur_rtde;

RTDEControlInterface rtde_control("172.30.10.1");
RTDEReceiveInterface rtde_receive("172.30.10.1");
int _num_samples;
std::string _chain_start, _chain_end, _urdf_param;
double _timeout;
double timestamp = 0;
bool CKDONE = false;
std::mutex mtx;
std::vector<double> eePos = {0,0,0,0,0,0,0,0,0,0,0,0};
bool modified = false;
std::thread thr;

void moveArm(std::vector<double> ee_pose, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param);

void setEEPos(std::vector<double> &p, double ts = 0)
{
  std::lock_guard<std::mutex> guard(mtx);
  eePos = p;
  modified = true;

  timestamp = ts;
}

static void loopCK()
{
  //std::vector<double> startPose = {-0.1,0.4,0.5, 2.938,-2.82,1.479};
  //std::vector<double> startPose = {-0.1, 0.4, 0.5, 4.356, -0.529, -0.587};
  //rtde_control.servoJ(rtde_control.getInverseKinematics(startPose), 1, 1, 0.1, 0.2, 300);
  printf("DEBUG:  loopCK\n");
  printf("eePos.size(): %lu \n", eePos.size());
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
        printf("DEBUG: modified\n");
        modified = false;
      }
    }
    printf("DEBUG: pre moveArm\n");
    printf("DEBUG: eePos.size(): %lu \n", eePos.size());
    moveArm(eePos, _num_samples, _chain_start, _chain_end, _timeout, _urdf_param);
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
  printf("Debug");
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
  //printf("DEBUG: newJointSpeed\n");
  std::vector<double> tmp_speed = vecSubtract(joint_config, actual_q);
  std::vector<double> abs_tmp_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < tmp_speed.size() - 1; i++)
  {
    abs_tmp_speed[i] = std::abs(tmp_speed[i]);
  }
  for (int i = 0; i < joint_config.size(); i++)
  {
    if (std::abs(tmp_speed[i]) > 0.002)
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
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

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

  std::cout << "Using" << chain.getNrOfJoints() << "joints";

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

  std::cout << "*** Testing TRAC-IK with " << num_samples << " random samples";

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
        std::cout << rc;
        printf("\n");
      }
      solutions.push_back(result);
    }
    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  writeToCsv(solutions);
  std::cout << "TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample";
}

KDL::JntArray calculateSolution(double num_samples, std::vector<double> startpos, std::vector<double> ee_pose, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  printf("DEBUG: calculateSolution\n");
  KDL::JntArray result;
  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps,TRAC_IK::Distance);
  printf("DEBUG: tracik_solver init done\n");
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

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
  //KDL::ChainFkSolverPos_recursive fk_solver(chain);                                     // Forward kin. solver
  //KDL::ChainIkSolverVel_pinv vik_solver(chain);                                         // PseudoInverse vel solver
  //KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

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

  KDL::Frame end_effector_pose;
  int rc;

  double total_time = 0;
  uint success = 0;

  std::cout << "*** TRAC-IK with: *** \n";
  std::cout << "Start\n";
  for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
  {
    std::cout << startpos[i] << std::endl;
  }

  printf("\n");
  KDL::Rotation rotation = KDL::Rotation(ee_pose[0], ee_pose[4], ee_pose[8], ee_pose[1], ee_pose[5], ee_pose[9], ee_pose[2], ee_pose[6], ee_pose[10]);
  KDL::Vector goal = KDL::Vector(ee_pose[3], ee_pose[7], ee_pose[11]);
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

  /*temp_goal(0) = goalpos[0];
  temp_goal(1) = goalpos[1];
  temp_goal(2) = goalpos[2];
  temp_goal(3) = goalpos[3];
  temp_goal(4) = goalpos[4];
  temp_goal(5) = goalpos[5];*/

  jnt.data = temp;
  jnt.resize(6);

  /*jnt_goal.data = temp_goal;
  jnt_goal.resize(6);*/

  //fk_solver.JntToCart(jnt_goal, end_effector_pose);
  std::cout << end_effector_pose << std::endl;
  rc = 0;

  double elapsed = 0;
  start_time = boost::posix_time::microsec_clock::local_time();
  printf("DEBUG: chain number of segments: %u\n", chain.getNrOfSegments());
  rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
  printf("DEBUG: chain number of segments #2: %u\n", chain.getNrOfSegments());
  diff = boost::posix_time::microsec_clock::local_time() - start_time;
  elapsed = diff.total_nanoseconds() / 1e9;
  std::cout << "Time needed for calculation:" << elapsed << std::endl;

  if (rc < 0)
  {
    printf("rc\n");
    //std:cout << rc << std::endl;
    printf("%i", rc);
    printf("\n");
  }
  else
  {
    printf("rc (no error):%i\n", rc);
    printf("Solution:\n");
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++)
    {
      std::cout << result(i) << std::endl;
    }
    //std::vector<double> goal = {result(0), result(1), result(2), result(3), result(4), result(5)};
    //rtde_control.moveJ(goal);
    printf("\n");
  }

  printf("Pose:\n %f, ", rtde_receive.getActualQ().at(0));
  printf("%f, ", rtde_receive.getActualQ().at(1));
  printf("%f, ", rtde_receive.getActualQ().at(2));
  printf("%f, ", rtde_receive.getActualQ().at(3));
  printf("%f, ", rtde_receive.getActualQ().at(4));
  printf("%f\n", rtde_receive.getActualQ().at(5));

  printf("TCP offset x: %f\n", rtde_receive.getActualTCPPose().at(0));
  printf("TCP offset y: %f\n", rtde_receive.getActualTCPPose().at(1));
  printf("TCP offset z: %f\n", rtde_receive.getActualTCPPose().at(2));
  printf("TCP offset rx: %f\n", rtde_receive.getActualTCPPose().at(3));
  printf("TCP offset ry: %f\n", rtde_receive.getActualTCPPose().at(4));
  printf("TCP offset rz: %f\n", rtde_receive.getActualTCPPose().at(5));

  return result;
}

/* move the arm with ur_rtde to the best found solution*/
void moveArm(std::vector<double> ee_pose, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  printf("DEBUG: moveArm\n");
  std::vector<double> actual_q = rtde_receive.getActualQ();

  KDL::JntArray result = calculateSolution(num_samples, actual_q, ee_pose, chain_start, chain_end, timeout, urdf_param);
  printf("\n");

  double sT = 0.002; //sample Time

  std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> goal = {result(0), result(1), result(2), result(3), result(4), result(5)};

  double max_vel = 3.14;
  double acceleration = 40.0;
  double dt = 0.02;
  actual_q = rtde_receive.getActualQ();

  printf("current joint angles:\n");
  for (int i = 0; i <= actual_q.size() - 1; i++)
    std::cout << actual_q[i] << " ";
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

  //move arm until the goal position is reached
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
  }

  actual_q = rtde_receive.getActualQ();

  printf("reached joint angles:\n");
  for (int i = 0; i <= actual_q.size() - 1; i++)
    std::cout << actual_q[i] << " ";
  printf("\n");


  rtde_control.speedStop();
  rtde_control.stopScript();
  rtde_receive.disconnect();

  printf("End Move Arm \n");
}

int main(int argc, char **argv)
{
  printf("DEBUG: main() #1\n");
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

  nh.param("timeout", _timeout, 0.005);
  nh.param("urdf_param", _urdf_param, std::string("/robot_description"));

  if (_num_samples < 1)
    _num_samples = 1;



  //testRandomSamples(num_samples, chain_start, chain_end, timeout, urdf_param);
  //moveArm("172.30.10.1", ee_pose, num_samples, chain_start, chain_end, timeout, urdf_param);

  // Useful when you make a script that loops over multiple launch files that test different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);

  // char **command = &commandVector[0];
  // execvp(command[0],command);
  std::vector<double> ee_pose = {-0.9993266, 0.0308951, -0.0197921, -0.15177,
                                -0.0157949, 0.1246475, 0.9920754, 0.4,
                                0.0331173, 0.9917200, -0.1240756, 0.236058};
  
  eePos = ee_pose;
  setEEPos(ee_pose);
  printf("DEBUG: main() #2\n");

  startCK();
  //moveArm(ee_pose, _num_samples, _chain_start, _chain_end, _timeout, _urdf_param);
  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}

/*
Error codes für rc (tracik_solver.CartToJnt):

-1 = E_DEGRADED 	
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


