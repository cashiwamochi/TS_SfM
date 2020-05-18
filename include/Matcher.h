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
        CheckType check_type;
        SearchType search_type;
        int search_range;
      };

      Matcher(const MatcherConfig _config);
      ~Matcher(){};

      std::vector<cv::DMatch>
        GetMatches(const Frame& frame0, const Frame& frame1);

      std::vector<cv::DMatch> Inverse(const std::vector<cv::DMatch>& v_matches) {
        std::vector<cv::DMatch> v_matches_inv = v_matches; 
        v_matches_inv.reserve(v_matches.size());
        for(cv::DMatch m : v_matches_inv) {
          std::swap(m.queryIdx, m.trainIdx);
          v_matches_inv.push_back(m);
        }
        return v_matches_inv;
      };

    private:
      std::unique_ptr<cv::BFMatcher> m_p_matcher;
      const MatcherConfig m_config;

      std::vector<cv::DMatch>
        GetMatchesByGridSearch(const Frame& frame0, const Frame& frame1, int neighbor = 1);
      std::vector<cv::DMatch>
        GetMatchesByRadiusSearch(const Frame& frame0, const Frame& frame1, int radius = 50);
       std::vector<cv::DMatch>
        GetMatchesByWholeSearch(const Frame& frame0, const Frame& frame1);

      std::vector<cv::DMatch>
        GetMatchesByEpipolarSearch(const Frame& frame0, const Frame& frame1);
      std::vector<cv::DMatch>
        GetMatchesUsingMotionModel(const Frame& frame0, const Frame& frame1);

      void ShowMatches(const Frame& frame0, const Frame& frame1, const std::vector<cv::DMatch>& v_matches_01);

  };

} // namespace
