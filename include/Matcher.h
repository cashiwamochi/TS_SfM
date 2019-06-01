#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>

namespace TS_SfM {

  class MapPoint;
  class Frame;

  class Matcher {
    public:

      enum CheckType {
        CrossCheck = 0,
        RatioTest = 1,
        CrossRatioCheck = 2
      };

      enum SearchType {
        Radius = 0,
        Grid = 1,
        Whole = 2
      };

      struct MatcherConfig {
        CheckType ckeck_type;
        SearchType search_type;
        int search_range;
      };

      Matcher(const CheckType& _checktype, const SearchType& _searchtype);
      ~Matcher(){};

      template<typename T> 
      std::vector<cv::DMatch>
        GetMatches(const Frame& frame0, const Frame& frame1, T value);

    private:
      std::unique_ptr<cv::BFMatcher> m_p_matcher;
      const CheckType m_checktype;
      const SearchType m_searchtype;

      std::vector<cv::DMatch>
        GetMatchesByGridSearch(const Frame& frame0, const Frame& frame1, int neighbor = 1);
      std::vector<cv::DMatch>
        GetMatchesByRadiusSearch(const Frame& frame0, const Frame& frame1, double radius = 50.0);
      std::vector<cv::DMatch>
        GetMatchesByWholeSearch(const Frame& frame0, const Frame& frame1);


  };

} // namespace
