#include "atl/utils/data.hpp"

namespace atl {

// ATTITUDE COMMAND
AttitudeCommand::AttitudeCommand(void) {
  this->orientation.w() = 0.0;
  this->orientation.x() = 0.0;
  this->orientation.y() = 0.0;
  this->orientation.z() = 0.0;
  this->throttle = 0.0;
}

AttitudeCommand::AttitudeCommand(Vec4 command) {
  // quaternion
  Vec3 euler{command(0), command(1), command(2)};  // roll, pitch, yaw
  euler2quat(euler, 321, this->orientation);

  // throttle
  this->throttle = command(3);
}

void AttitudeCommand::print(void) {
  Vec3 euler;
  quat2euler(this->orientation, 321, euler);

  printf("roll: %.2f\t", euler(0));
  printf("pitch: %.2f\t", euler(1));
  printf("yaw: %.2f\t", euler(2));
  printf("throttle: %.2f\n", this->throttle);
}

// POSE
Pose::Pose(
  double roll, double pitch, double yaw, double x, double y, double z) {
  Vec3 euler;
  euler << roll, pitch, yaw;
  euler2quat(euler, 321, this->orientation);
  this->position << x, y, z;
}

Mat3 Pose::rotationMatrix() {
  return this->orientation.toRotationMatrix();
}

void Pose::printPosition() {
  std::cout << "position [";
  std::cout << std::setprecision(2) << this->position(0);
  std::cout << std::setprecision(2) << this->position(1);
  std::cout << std::setprecision(2) << this->position(2);
  std::cout << "]" << std::endl;
}

void Pose::printOrientation() {
  std::cout << "quaternion[";
  std::cout << std::setprecision(2) << this->orientation.w();
  std::cout << std::setprecision(2) << this->orientation.x();
  std::cout << std::setprecision(2) << this->orientation.y();
  std::cout << std::setprecision(2) << this->orientation.z();
  std::cout << "]" << std::endl;
}

void Pose::print() {
  this->printPosition();
  this->printOrientation();
}

// CSV
int csvrows(std::string file_path) {
  int nb_rows;
  std::string line;
  std::ifstream infile(file_path);

  // load file
  if (infile.good() != true) {
    printf(E_CSV_DATA_LOAD, file_path.c_str());
    return -1;
  }

  // obtain number of lines
  nb_rows = 0;
  while (std::getline(infile, line)) {
    nb_rows++;
  }

  return nb_rows;
}

int csvcols(std::string file_path) {
  int nb_elements;
  std::string line;
  bool found_separator;
  std::ifstream infile(file_path);

  // setup
  nb_elements = 1;
  found_separator = false;

  // load file
  if (infile.good() != true) {
    printf(E_CSV_DATA_LOAD, file_path.c_str());
    return -1;
  }

  // obtain number of commas
  std::getline(infile, line);
  for (size_t i = 0; i < line.length(); i++) {
    if (line[i] == ',') {
      found_separator = true;
      nb_elements++;
    }
  }

  return (found_separator) ? nb_elements : 0;
}

int csv2mat(std::string file_path, bool header, MatX &data) {
  int line_no;
  int nb_rows;
  int nb_cols;
  std::string line;
  std::ifstream infile(file_path);
  std::vector<double> vdata;
  std::string element;
  double value;

  // load file
  if (infile.good() != true) {
    printf(E_CSV_DATA_LOAD, file_path.c_str());
    return -1;
  }

  // obtain number of rows and cols
  nb_rows = csvrows(file_path);
  nb_cols = csvcols(file_path);

  // header line?
  if (header) {
    std::getline(infile, line);
    nb_rows -= 1;
  }

  // load data
  line_no = 0;
  data.resize(nb_rows, nb_cols);
  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    // load data row
    for (int i = 0; i < nb_cols; i++) {
      std::getline(ss, element, ',');
      value = atof(element.c_str());
      data(line_no, i) = value;
    }

    line_no++;
  }

  return 0;
}

int mat2csv(std::string file_path, MatX data) {
  std::ofstream outfile(file_path);

  // open file
  if (outfile.good() != true) {
    printf(E_CSV_DATA_OPEN, file_path.c_str());
    return -1;
  }

  // save matrix
  for (int i = 0; i < data.rows(); i++) {
    for (int j = 0; j < data.cols(); j++) {
      outfile << data(i, j);

      if ((j + 1) != data.cols()) {
        outfile << ",";
      }
    }
    outfile << "\n";
  }

  // close file
  outfile.close();
  return 0;
}

}  // namespace atl
