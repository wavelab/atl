#include "awesomo/ekf.hpp"


void ekf_prediction_update(struct ekf *estimator, u)
{
	g = estimator->g_function(u, ekf->mu, ekf->dt);
	G = estimator->G_function(u, ekf->mu, ekf->dt);

	estimator->mu_p = g;
	estimator->S_p = G * estimator->S * G.transpose() + estimator->R;
}

void ekf_measurement_update(struct ekf *estimator, h_func, H_func, y)
{
	h = estimator->h_function(estimator->mu_p);
	H = estimator->H_function(estimator->mu_p);

	// K = estimator->S_p * H.transpose() * inv(H * estimator->S_p * H.transpose() + estimator->Q);
	estimator->mu = estimator->mu_p + K * (y - h);
    // estimator->S = (eye(length(estimator->mu)) - K * H) * estimator->S_p;
}
