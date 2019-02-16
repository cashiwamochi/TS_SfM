#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "System.h"

using std::cout;
using std::endl;

void showUsage();

int main(int argc, char* argv[]) {
  if(argc != 3) {
    showUsage();
    return -1;
  }


  return 0;
}


void showUsage() {
  cout << "Usage : this.out [/path/to/camera_params.yaml] "
       << "[/path/to/config_params.yaml] "
       << endl;
  return;
}
