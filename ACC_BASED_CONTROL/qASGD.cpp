//////////////////////////////////////////\/////////\//
// Leonardo Felipe Lima Santos dos Santos /\     ////\/
// leonardo.felipe.santos@usp.br	_____ ___  ___  //|
// github/bitbucket qleonardolp /	| |  | . \/   \  /|
// *Copyright 2021-2026* \//// //  	| |   \ \   |_|  /|
//\///////////////////////\// ////	\_'_/\_`_/__|   ///
///\///////////////////////\ //////////////////\////\//

#ifdef _WIN32
#include "QpcLoopTimer.h" // ja inclui <windows.h>
#else
#include <windows.h>
#endif
#include "SharedStructs.h" // ja inclui <stdio.h> / <thread> / <mutex> / <vector>
#include "LowPassFilter2p.h"
#include <processthreadsapi.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>

/*  -- Obtain attitude quaternion:
    -- Quaternion-based Attitude estimation using ASGD algorithm
    -- Refs: 
    -- [1]: Quaternion-based Kalman filter for AHRS using an adaptive-step gradient descent algorithm, 
    --      Wang, Li and Zhang, Zheng and Sun, Ping, International Journal of Advanced Robotic Systems, 2015
    -- [2]: Estimation of IMU and MARG orientation using a gradient descent algorithm,
    --      Madgwick, Sebastian OH and Harrison, Andrew JL and Vaidyanathan, Ravi, international Conference on Rehabilitation Robotics, 2011
    -- [3]: "How to integrate Quaternions", Ashwin Narayan (www.ashwinnarayan.com/post/how-to-integrate-quaternions/)
*/

void qASGD(ThrdStruct &data_struct){
    //...
}