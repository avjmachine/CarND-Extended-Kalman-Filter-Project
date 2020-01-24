#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0.,0.,0.,0.;
    for(int i=0;i<estimations.size();i++)
        {
	    VectorXd residual = estimations[i] - ground_truth[i];
	    VectorXd sq_residual = residual.array() * residual.array();  
            rmse += sq_residual;	    
        }
    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd J(3,4);
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float sq_res_pos = px*px + py*py;
    float res_pos = sqrt(sq_res_pos);    
    float res_pos_pow3by2 = sq_res_pos * res_pos;

    if (fabs(res_pos)<0.0001)
    {
	std::cout<<"Division by zero! Returning a Jacobian with a minimum assumed resultant position value\n";
        res_pos = 0.0001;
	sq_res_pos = res_pos * res_pos;
	res_pos_pow3by2 = sq_res_pos * res_pos;
    }

    J(0,0) = px/res_pos; 
    J(0,1) = py/res_pos;
    J(0,2) = 0.;
    J(0,3) = 0.;

    J(1,0) = -py/sq_res_pos;
    J(1,1) = px/sq_res_pos;
    J(1,2) = 0.;
    J(1,3) = 0.;

    J(2,0) = py*(vx*py - vy*px)/res_pos_pow3by2;
    J(2,1) = px*(vy*px - vx*py)/res_pos_pow3by2;
    J(2,2) = px/res_pos;
    J(2,3) = py/res_pos;

    return J;
}
