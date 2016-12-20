

void ekf_prediction_update(struct ekf *e, Eigen::VectorXd u, float dt) {
  Eigen::MatrixXd g;
  Eigen::MatrixXd G;

  g = e->g_function(u, e->mu, dt);
  G = e->G_function(u, e->mu, dt);

  e->mu_p = g;
  e->S_p = G * e->S * G.transpose() + e->R;
}

void ekf_measurement_update(struct ekf *e, Eigen::VectorXd y, float dt) {
  Eigen::VectorXd h;
  Eigen::MatrixXd H;

  h = e->h_function(e->mu_p, dt);
  H = e->H_function(e->mu_p, dt);

  // clang-format off
  e->K = e->S_p * H.transpose() * (H * e->S_p * H.transpose() + e->Q).inverse();
  e->mu = e->mu_p + e->K * (y - h);
  e->S = (e->I - e->K * H) * e->S_p;
  // clang-format on
}
