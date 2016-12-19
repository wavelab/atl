#include <iostream>
#include "awesomo/estimator.hpp"


void apriltag_kf_setup(struct kf *e, Eigen::VectorXd mu)
{
    Eigen::MatrixXd A(9, 9);
    Eigen::MatrixXd B(9, 9);
    Eigen::MatrixXd R(9, 9);

    Eigen::MatrixXd C(3, 9);
    Eigen::MatrixXd Q(3, 3);

    Eigen::MatrixXd S(9, 9);
    Eigen::MatrixXd I(9, 9);
    Eigen::MatrixXd K(9, 9);

	Eigen::VectorXd mu_p(9);
	Eigen::MatrixXd S_p(9, 9);

	// transition matrix (assuming constant acceleration)
	A << 1, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 1, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 1, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 1, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 1, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 1;

	// input matrix
	B = Eigen::MatrixXd::Zero(9, 9);

    // motion noise
	R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 0.5, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 0.5, 0, 0, 0, 0, 0, 0,
		 0, 0, 0, 1.0, 0, 0, 0, 0, 0,
		 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
		 0, 0, 0, 0, 0, 1.0, 0, 0, 0,
		 0, 0, 0, 0, 0, 0, 1.0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 1.0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 1.0;

	// measurement model
	C << 1, 0, 0, 0, 0, 0, 0, 0, 0,
		 0, 1, 0, 0, 0, 0, 0, 0, 0,
		 0, 0, 1, 0, 0, 0, 0, 0, 0;

    // measurement noise
	Q << 20, 0, 0,
		 0, 20, 0,
		 0, 0, 20;

	// misc
	S = Eigen::MatrixXd::Identity(9, 9) * 100;
	I = Eigen::MatrixXd::Identity(9, 9);
	K = Eigen::MatrixXd::Zero(9, 9);
	// mu_p = Eigen::VectorXd::Zero(9);
	S_p = Eigen::MatrixXd::Zero(9, 9);

    // configure kalman filter
    e->mu = mu;

    e->A = A;
    e->B = B;
    e->R = R;

    e->C = C;
    e->Q = Q;

    e->S = S;
    e->I = I;
    e->K = K;

    e->mu_p = mu_p;
    e->S_p = S_p;
}

void apriltag_kf_estimate(struct kf *e, Eigen::VectorXd y, float dt, bool tag_detected)
{
	// transition matrix (constant acceleration)
	e->A <<
	     1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0, 0,
		 0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0,
		 0, 0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0,
		 0, 0, 0, 1.0, 0, 0, dt, 0, 0,
		 0, 0, 0, 0, 1.0, 0, 0, dt, 0,
		 0, 0, 0, 0, 0, 1.0, 0, 0, dt,
		 0, 0, 0, 0, 0, 0, 1.0, 0, 0,
		 0, 0, 0, 0, 0, 0, 0, 1.0, 0,
		 0, 0, 0, 0, 0, 0, 0, 0, 1.0;

	// transition matrix (constant velocity)
	// e->A << 1, 0, 0, dt, 0, 0, 0, 0, 0,
	// 	 0, 1, 0, 0, dt, 0, 0, 0, 0,
	// 	 0, 0, 1, 0, 0, dt, 0, 0, 0,
	// 	 0, 0, 0, 1, 0, 0, 0, 0, 0,
	// 	 0, 0, 0, 0, 1, 0, 0, 0, 0,
	// 	 0, 0, 0, 0, 0, 1, 0, 0, 0,
	// 	 0, 0, 0, 0, 0, 0, 0, 0, 0,
	// 	 0, 0, 0, 0, 0, 0, 0, 0, 0,
	// 	 0, 0, 0, 0, 0, 0, 0, 0, 0;

	// prediction update
    e->mu_p = e->A * e->mu;
    e->S_p = e->A * e->S * e->A.transpose() + e->R;

	// measurement update
	if (tag_detected) {
		e->K = e->S_p * e->C.transpose() * (e->C * e->S_p * e->C.transpose() + e->Q).inverse();
		e->mu = e->mu_p + e->K * (y - e->C * e->mu_p);
		e->S = (e->I - e->K * e->C) * e->S_p;

	} else {
		e->mu = e->mu_p;
		e->S = e->S_p;

	}

}

void ekf_prediction_update(struct ekf *e, Eigen::VectorXd u, float dt)
{
    Eigen::MatrixXd g;
    Eigen::MatrixXd G;

    g = e->g_function(u, e->mu, dt);
    G = e->G_function(u, e->mu, dt);

    e->mu_p = g;
    e->S_p = G * e->S * G.transpose() + e->R;
}

void ekf_measurement_update(struct ekf *e, Eigen::VectorXd y, float dt)
{
    Eigen::VectorXd h;
    Eigen::MatrixXd H;

    h = e->h_function(e->mu_p, dt);
    H = e->H_function(e->mu_p, dt);

    e->K = e->S_p * H.transpose() * (H * e->S_p * H.transpose() + e->Q).inverse();
    e->mu = e->mu_p + e->K * (y - h);
    e->S = (e->I - e->K * H) *  e->S_p;
}
