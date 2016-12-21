#include "awesomo_core/utils/io.hpp"

namespace awesomo {

bool file_exists(const std::string &fp) {
  FILE *file;

  if (file = fopen(fp.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

}  // end of awesomo namepsace
