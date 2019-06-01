#pragma once

#include <iostream>
#include <vector>

namespace TS_SfM {

  class MapPoint;
  class Frame;

  class Matcher {
    public:
      Matcher();
      ~Matcher(){};

      enum CheckType {
        CrossCheck = 0,
        RatioTest = 1,
        CrossRatioCheck = 2
      };

      enum SearchType {
        Radius = 0,
        Grid = 1
      };

    // private:


  };

} // namespace
