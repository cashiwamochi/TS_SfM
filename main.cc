#include <iostream>

#include "System.h"

using std::cout;
using std::endl;

void ShowUsage();

int main(int argc, char* argv[]) {
  if(argc != 2) {
    ShowUsage();
    return -1;
  }

  const std::string str_config_file = argv[1];
  TS_SfM::System _sfm(str_config_file);

  _sfm.Run();

  return 0;
}


void ShowUsage() {
  cout << "Usage : this.out [/path/to/config_params.yaml] "
       << endl;
  return;
}
