/**
 *ROJITHA GOONESEKERE - MASTERS - UNIVERSITY OF NOTRE DAME
 *This is part of my main body of work during my masters
 *This controller represents part of my novel proprioceptive contact detector
 *The controller allows the MIT cheetah quadruped robot to navigate without external sensing
 */


#include "MIT_Controller.hpp"
#include <lcm/lcm-cpp.hpp>
#include "/Users/rojitha/Documents/Cheetah-Software-RG/lcm-types/cpp/MIT_stream.hpp"

MIT_Controller::MIT_Controller():RobotController(){
}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters,
                                      _visualizationData, &userParameters);
  _controlParameters->control_mode = 6;
    
  filter_state.setZero();
  filter_stated.setZero();
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {
    
    lcm::LCM lcm;
    
    MIT_stream ContactImpl;


    // Find the current gait schedule
  _gaitScheduler->step();

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
    

     FBModelState<float> state;
    
     state.bodyOrientation = _stateEstimate->orientation;
     state.bodyPosition = _stateEstimate->position;
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

     /////// Contact detection test///////

    //Initialize bfric & mufric
//    Eigen::Matrix3f bfric;
//    bfric << 0.1, 0, 0,
//             0, 0.1, 0,
//             0, 0, 0.6;
    
    float bfric = 0;
    float mufric = 0;
 // Eigen::Matrix3f mufric = Eigen::Matrix3f::Identity();

    Vec18<float> tau;
    tau.setZero();

    
    Vec12<float> Foot_f;
    Foot_f.setZero();
    Vec12<float> Foot_fd;
    Foot_fd.setZero();

    float side_sign;
    Eigen::Vector3f sign(0, 0, 0);
    Eigen::Vector3f taustore(0, 0, 0);
    Eigen::Vector3f first(0, 0, 0);
    Eigen::Vector3f sec(0, 0, 0);
    Eigen::Vector3f third(0, 0, 0);
    Eigen::Vector3f tau_leg(0, 0, 0);
    Eigen::Vector3f Foot_f_int(0, 0, 0);
    Eigen::Vector3f tau_legd(0, 0, 0);
    Eigen::Vector3f Foot_f_intd(0, 0, 0);
    
    for (int i = 0; i<4; i++)
    {
        int i1 = (6 + 3*(i-1) + 3);
         
            for (int c = 0; c < 3; c++)
            {
                sign(c) = ((_legController->datas[i].qd[c] > 0) ? 1 : -1);
            }
        
        first = (_legController->datas[i].tauEstimate.cast<float>());
        sec = (mufric*sign);
        third = (bfric*_legController->datas[i].qd.cast<float>());
        taustore = first - sec - third;
        tau.segment(i1,3) = taustore.segment(0,3);
        
        for (int u=0; u<3; u++)
        {
            ContactImpl.tau[i][u] = taustore(u);
            ContactImpl.tau_estimate[i][u] = first(u);
        }
        
    }

    double pi = 3.14159265358979323846;
    double lambda = 2*pi*5; //continous time implimentation
    //double lambda = 40;
    //double lambdaC = 0.1; //Lambda for continous time disturbance observer in discrete time
    //double discrete_FIR = 0.4904;
    double gamma;
    double st = 0.002; //sampling time for the mini cheetah
    gamma = exp(-lambda*st);
    double beta;
    beta = ((1-gamma)*pow(gamma,-1))/st; //discrete time implementation
    Vec18<float> pp,taud_est, term_to_filter, ppd, taud_estd, term_to_filterd, nu, termint;
    pp.setZero();
    ppd.setZero();
    taud_est.setZero();
    taud_estd.setZero();
    term_to_filter.setZero();
    term_to_filterd.setZero();
    nu.setZero();
    termint.setZero();

    nu.head(3) = _stateEstimate->omegaBody;
    nu.segment(3,3) = _stateEstimate->vBody;
    nu.tail(12) = state.qd;
    
    //Discrete time distubance observer
    ppd = beta*_model->massMatrix()*nu;
    term_to_filterd = ppd + _model->coriolisMatrix().transpose()*nu - _model->generalizedGravityForce() + tau;
    filter_stated = (1-gamma)*term_to_filterd + gamma*filter_stated;
    taud_estd = ppd - filter_stated;
    
    //Continuous time disturbance observer in discrete time
    pp = lambda*_model->massMatrix()*nu;
    term_to_filter = pp + _model->coriolisMatrix().transpose()*nu - _model->generalizedGravityForce() + tau;
    filter_state = (1-gamma)*term_to_filter + gamma*filter_state;
    taud_est = pp - filter_state;

    Vec4<float> f_z;
    f_z.setZero();
    
    Vec4<float> f_zd;
    f_zd.setZero();
    
    Eigen::Matrix3f Jf(3,3);
    Eigen::Matrix3f Jf_T(3,3);
    Eigen::Matrix3f Jf_T_Tol(3,3);
    Eigen::Matrix3f Jf_T2(3,3);
    
    _model->contactJacobians();
    
    for (int n = 0; n < 4; n++)
    {
        int i11 = (6 + 3*(n-1) + 3);
        int i3 = 3*(n-1) + 3;

        D3Mat<float> Jf_full = _model->getFootJacobian(n);
        
        Jf = Jf_full.template block<3, 3>(0,i11);

        Jf_T = Jf.transpose();
        
        //counter1++;
        
        Jf_T2 = Jf_T.completeOrthogonalDecomposition().pseudoInverse();
        
        for (int p=0; p<3; p++)
               {
                   for (int h=0; h<3; h++)
                   {
                       if (Jf_T2(p,h) < 0.01 && Jf_T2(p,h) > 0)
                                        {
                                            Jf_T2(p,h) = 0;
                                        }
                   }

               }
        
        //Discrete time version
        tau_legd.segment(0,3) = taud_estd.segment(i11, 3);
        Foot_f_intd = Jf_T2*tau_legd;
        Foot_fd.segment(i3,3) = Foot_f_intd.segment(0,3);
        f_zd(n) = Foot_fd(i3+2);
        
        //continuous time discretized version
        tau_leg.segment(0,3) = taud_est.segment(i11, 3);
        Foot_f_int = Jf_T2*tau_leg;
        Foot_f.segment(i3,3) = Foot_f_int.segment(0,3);
        f_z(n) = Foot_f(i3+2);
        
        
             for (int h=0; h<3; h++)
                       {
                           ContactImpl.Foot_f[n][h] = Foot_f_int(h);
                           ContactImpl.tau_leg[n][h] = tau_leg(h);
                           ContactImpl.Foot_fd[n][h] = Foot_f_intd(h);
                           ContactImpl.tau_legd[n][h] = tau_legd(h);
                       }



//DEBUG HERE//

//        cout << "Counter = " << counter1 << "\n" << "\n";
//        cout << "nu =" << "\n" << nu << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Force f =" << "\n" << Foot_f << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Force f =" << "\n" << f_z << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "tau_leg =" << "\n" <<  first << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Itr" << y << "\n";
//        cout << "Jf_T2 =" << "\n" <<  Jf_T2 << "\n" << "\n" << "\n" << "\n";
//        cout << "tau = " << "\n" << tau_leg << "\n" << "\n" << "\n" << "\n";
//        cout << "Force f int =" << "\n" << Foot_f_int << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Jf_full =" << "\n" <<  Jf_full << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Foot Jacobian = " << "\n" << _model->getFootJacobian(n) << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "MM = " << "\n" << _model->massMatrix() << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Grav = " << "\n" << _model->generalizedGravityForce() << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "taudest = " << "\n" << taud_est << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Coriolis = " << "\n" << _model->generalizedCoriolisForce() << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Coriolis = " << "\n" << _model->coriolisMatrix() << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Term to filter = " << "\n" << term_to_filter << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//        cout << "Filter State = " << "\n" << filter_state << "\n" << "\n" << "\n" << "\n";
//        cout << "____________" << "\n" << "\n";
//

    }
    
    lcm.publish("Contact_Algo_Data", &ContactImpl);
      
    /////// End Contact Detection Algo Test///////
 
}
