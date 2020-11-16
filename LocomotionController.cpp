/**
 *ROJITHA GOONESEKERE - MASTERS - UNIVERSITY OF NOTRE DAME
 *This is part of my main body of work during my masters
 *This controller represents a test run of robot control functionality
 *The controller allows the MIT cheetah quadruped robot to showcase it's locomotion functionality
 */


#include "LocomotionController.hpp"
#include <lcm/lcm-cpp.hpp>
#include "exlcm/InvDyn_stream.hpp"
#include <iostream>
#include <cmath>

void Leg_InvDyn_Controller::runController(){
 
    
  static int iter = 0;
  iter ++;

  // This section eventually won't be needed after state estimate
  // is auto propagated to the model
  // {
  
  FBModelState<float> state;
  
  state.bodyOrientation = _stateEstimate->orientation;
  state.bodyPosition    = _stateEstimate->position;
  state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
  state.bodyVelocity.tail(3) = _stateEstimate->vBody;

  state.q.setZero(12);
  state.qd.setZero(12);

  for (int i = 0; i < 4; ++i) {
    state.q(3*i+0) = _legController->datas[i].q[0];
    state.q(3*i+1) = _legController->datas[i].q[1];
    state.q(3*i+2) = _legController->datas[i].q[2];
    state.qd(3*i+0)= _legController->datas[i].qd[0];
    state.qd(3*i+1)= _legController->datas[i].qd[1];
    state.qd(3*i+2)= _legController->datas[i].qd[2];
  }
  _model->setState(state);
  // }

  float t = _controlParameters->controller_dt*iter;

  // Desired trajectory parameters for a simple joint-space trajectory
  float freq_Hz = 1;
  float freq_rad = freq_Hz * 2* 3.14159;
  float amplitude = 3.1415/3;


  // Desired angles, angular velocities, and angular accelerations. 
  // We'll be lazy and use the same desired trajectory for each moving joint.
  Vec12<float> qDes, qdDes, qddDes;
  qDes.setZero();
  qdDes.setZero();
  qddDes.setZero();

  // Desired trajecotry for each joint is a sin wave
  float desired_angle = sin(t*freq_rad)*amplitude;
  float desired_rate = freq_rad*cos(t*freq_rad)*amplitude;
  float desired_acceleration = freq_rad*freq_rad*sin(t*freq_rad)*amplitude;

  // Set desired for a subset of legs
  for( int i = 0 ; i < (int) userParameters.num_moving_legs ; i++) {
    qDes.segment(3*i,3).setConstant(desired_angle);
    qdDes.segment(3*i,3).setConstant(desired_rate);
    qddDes.segment(3*i,i).setConstant(desired_acceleration);
  }

  // Opposite Ab/ad for L & R Legs
  qDes(0)*=-1; qdDes(0)*=-1; qddDes(0)*=-1;
  qDes(6)*=-1; qdDes(6)*=-1; qddDes(6)*=-1;


  // Construct commanded acceleration
  FBModelStateDerivative<float> commandedAccleration;
  commandedAccleration.dBodyVelocity.setZero();
  commandedAccleration.qdd = qddDes + 25*(qdDes - state.qd) + 150*(qDes - state.q);
 
  // Run RNEA inverse dynamics
  Vec18<float> generalizedForce = _model->inverseDynamics(commandedAccleration); // [base_force ; joint_torques]
  Vec12<float> jointTorques = generalizedForce.tail(12); // prune away base force


  // Alternate strategy: Assemble equations of motion
  Mat18<float> H;
  Vec18<float> Cqd, tau_grav;
  
  H = _model->massMatrix();
  Cqd = _model->generalizedCoriolisForce();
  tau_grav = _model->generalizedGravityForce();

  Vec18<float> generalizedAcceleration;
  generalizedAcceleration.head(6)  = commandedAccleration.dBodyVelocity;
  generalizedAcceleration.tail(12) = commandedAccleration.qdd;
  Vec18<float> generalizedForce2 = H*generalizedAcceleration + Cqd + tau_grav;

  // Make sure they match
  Vec18<float> err = generalizedForce - generalizedForce2;
  assert( err.norm() < 1e-4 );

  // Send joint torques to leg controllers
  int dof = 0;
  for(int leg(0); leg<4; ++leg){
    for(int jidx(0); jidx<3; ++jidx){
      _legController->commands[leg].qDes[jidx] = 0;
      _legController->commands[leg].qdDes[jidx] = 0.;
      _legController->commands[leg].tauFeedForward[jidx] = jointTorques(dof);
      dof++;
    }
    _legController->commands[leg].kpJoint = Mat3<float>::Zero();
    _legController->commands[leg].kdJoint = Mat3<float>::Zero();
  }

     /////// Contact Detection Algo Test///////

    //Initialize bfric & mufric
    Eigen::Matrix3f bfric;
    bfric << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.6;

    Eigen::Matrix3f mufric = Eigen::Matrix3f::Identity();

    Vec18<float> tau;
    tau.setZero();

    float side_sign;
    Eigen::Vector3f sign(0, 0, 0);
    Eigen::Vector3f first(0, 0, 0);
    Eigen::Vector3f sec(0, 0, 0);
    Eigen::Vector3f third(0, 0, 0);
    
    
    for (int i = 0; i<4; i++)
    {
        side_sign = pow(-1.0, i);

        for (int i1 = (6 + 3*(i-1) + 1); i1 < i1+3; i1++)
        {
            for (int c = 1; c < 4; c++)
            {
                sign(c) = float(((_legController->datas[i].qd[c] > 0) ? 1 : -1));
            }
            
            first = (side_sign*(_legController->datas[i].tauEstimate.cast<float>()));
            sec = (mufric*sign);
            third = (bfric*_legController->datas[i].qd.cast<float>());
                        
            tau(i1) = first(i) - sec(i) - third(i);

        }
    }

    double pi = 3.14159265358979323846;
    double lambda = 2*pi*100;
    double discrete_FIR = 0.4904;
    Vec18<float> pp, taud_est, term_to_filter, filter_state;
    pp.setZero();
    taud_est.setZero();
    term_to_filter.setZero();
    filter_state.setZero();
    double tau_leg = 0;

    pp = lambda*(_model->getMassMatrix()*state.qd);
    term_to_filter = pp + _model->getCoriolisForce()*state.qd - (_model->getGravityForce() + tau);
    filter_state = discrete_FIR*term_to_filter + (1-discrete_FIR)*filter_state;
    taud_est = pp - filter_state;

    Vec4<float> f_z;
    f_z.setZero();
    
    Eigen::Matrix3f Jf(3,3);
    Eigen::Matrix3f Jf_T(3,3);
    Eigen::Matrix3f Foot_f(3,3);
    
    
    for (int n = 1; n < 5; n++)
    {
        int i1 = 6 + 3*(n-1) + 1;
        int i3 = 3*(n-1) + 1;
        int i4 = 3*n;

        D3Mat<float> Jf_full = _model->getFootJacobian(n);
        
        Jf = Jf_full.template block<3, 3>(0,0);

        Jf_T = Jf.transpose();

    
        for (int ii = 1; ii <= 2; ++ii)
        {
        tau_leg = taud_est(i1);
        }
        
        Foot_f = Jf_T.completeOrthogonalDecomposition().pseudoInverse()*tau_leg;
    /*
        for (int i=1; i<4; i++)
        {
            InvDyn_data.Foot_f[i][i] = Foot_f(i,i);
        }
      
        lcm.publish("Foot force", &InvDyn_data);
    */
        
        f_z(n,1) = Foot_f(i3+2);
    }
    
    /////// End Contact Detection Algo Test///////
    
}
 
