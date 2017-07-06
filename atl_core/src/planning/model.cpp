#include "atl/planning/model.hpp"

namespace atl {

// QUAD2DMODEL
Quad2DModel::Quad2DModel(void) {
  this->configured = false;

  this->x << 0.0, 0.0, 0.0, 0.0;
  this->m = 0.0;
}

int Quad2DModel::configure(Vec4 x_init, double m) {
  this->x = x_init;
  this->m = m;

  this->configured = true;
  return 0;
}

int Quad2DModel::update(Vec2 u, double dt) {
  const double g = 9.81;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // update
  // x1 - x
  // x2 - vx
  // x3 - z
  // x4 - vz
  // u1 - az
  // u2 - theta
  this->x(0) = this->x(0) + this->x(1) * dt;
  this->x(1) = this->x(1) + u(0) * sin(u(1)) * dt;
  this->x(2) = this->x(2) + this->x(3) * dt;
  this->x(3) = this->x(3) + (u(0) * cos(u(1)) - g) * dt;

  return 0;
}

void Quad2DModel::printState(void) {
  std::cout << "x: " << this->x(0) << std::endl;
  std::cout << "vx: " << this->x(1) << std::endl;
  std::cout << "z: " << this->x(2) << std::endl;
  std::cout << "vz: " << this->x(3) << std::endl;
}

// SIMULATOR
Simulator::Simulator(void) {
  this->configured = false;
  this->model = Quad2DModel();
  this->x_init << 0.0, 0.0, 0.0, 0.0;
  this->x_final << 0.0, 0.0, 0.0, 0.0;

  this->d_az = 0.0;
  this->d_theta = 0.0;
  this->az_sum = 0.0;
  this->dist_error = 0.0;
  this->vel_error = 0.0;
}

int Simulator::configure(Vec4 x_init, Vec4 x_final, double m) {
  this->model.configure(x_init, m);
  this->x_init = x_init;
  this->x_final = x_final;

  this->d_az = 0.0;
  this->d_theta = 0.0;
  this->az_sum = 0.0;
  this->dist_error = 0.0;
  this->vel_error = 0.0;

  this->configured = true;
  return 0;
}

int Simulator::simulate(double dt, double tend, MatX U, MatX &X) {
  Vec2 u, u_prev;
  int nb_ts;

  // setup
  this->model.x = this->x_init;
  nb_ts = tend / dt;
  this->d_az = 0.0;
  this->d_theta = 0.0;
  this->az_sum = 0.0;
  this->dist_error = 0.0;
  this->vel_error = 0.0;
  X.resize(4, nb_ts);

  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (U.cols() != nb_ts || U.rows() != 2) {
    return -2;
  }

  // simulate
  for (int i = 0; i < nb_ts; i++) {
    u = U.col(i);
    model.update(u, dt);
    X.block(0, i, 4, 1) = model.x;

    // record energy used in terms of thrust and pitch change
    if (i != 0) {
      u_prev = U.col(i - 1);
      this->d_az += fabs(u_prev(0) - u(0));
      this->d_theta += fabs(u_prev(1) - u(1));
      this->az_sum += fabs(u(0));
    }
  }

  // calculate error against final state
  this->dist_error += pow((model.x(0) - this->x_final(0)), 2);  // x
  this->dist_error += pow((model.x(2) - this->x_final(2)), 2);  // z
  this->vel_error += pow((model.x(1) - this->x_final(1)), 2);   // vx
  this->vel_error += pow((model.x(3) - this->x_final(3)), 2);   // vz

  return 0;
}

}  // namespace atl
