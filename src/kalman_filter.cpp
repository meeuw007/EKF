#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	/**
	TODO:
	* predict the state
	*/
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Kalman Filter equations
	*/
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	x_ = x_ + (K * y);
	long n = x_.size();
	MatrixXd I = MatrixXd::Identity(n, n);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Extended Kalman Filter equations
	*/
	float xx = x_(0);
	float yy = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float rho = sqrt(xx * xx + yy * yy);
	float theta = atan2(yy, xx);
	float rho_dot;
	if (fabs(rho) < 0.0001) {
		rho_dot = 0;


	}
	else {

		rho_dot = (xx * vx + yy * vy) / rho;
	}

	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, rho_dot;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	y(1) = std::atan2(sin(y(1)), cos(y(1)));

	x_ = x_ + (K * y);
	long n = x_.size();
	MatrixXd I = MatrixXd::Identity(n, n);
	P_ = (I - K * H_) * P_;
}
