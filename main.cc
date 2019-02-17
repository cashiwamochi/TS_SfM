#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "System.h"

using std::cout;
using std::endl;

void showUsage();

int main(int argc, char* argv[]) {
  if(argc != 2) {
    showUsage();
    return -1;
  }

  const std::string str_config_file = argv[1];
  TS_SfM::System _sfm(str_config_file);

  return 0;
}


void showUsage() {
  cout << "Usage : this.out [/path/to/config_params.yaml] "
       << endl;
  return;
}
