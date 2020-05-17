#include "Matcher.h"
#include "Frame.h"


namespace TS_SfM {
  
  Matcher::Matcher(const MatcherConfig _config)
    : m_config(_config)
  {

    if(m_config.check_type == CrossCheck) {
      m_p_matcher.reset(new cv::BFMatcher(cv::NORM_HAMMING, true));
    }
    else {
      m_p_matcher.reset(new cv::BFMatcher(cv::NORM_HAMMING, false));
    }
   
  }


  std::vector<cv::DMatch>
    Matcher::GetMatches(const Frame& frame0, const Frame& frame1) 
    {
      std::vector<cv::DMatch> v_matches;
      switch(m_config.search_type) {
        case Radius:
          v_matches = GetMatchesByRadiusSearch(frame0, frame1, (double)m_config.search_range);
          break;
        case Grid:
          v_matches = GetMatchesByGridSearch(frame0, frame1, m_config.search_range);
          break;
        case Whole:
          v_matches = GetMatchesByWholeSearch(frame0, frame1);

          break;
        default:
          break;
      } 

      if(false) {
        ShowMatches(frame0,frame1,v_matches);
      }

      return v_matches;
    }

  std::vector<cv::DMatch> Matcher::GetMatchesByGridSearch(const Frame& frame0, const Frame& frame1, int neighbor)
  {
    std::vector<cv::DMatch> v_matches; 


    return v_matches;
  }

  std::vector<cv::DMatch> Matcher::GetMatchesByRadiusSearch(const Frame& frame0, const Frame& frame1, int radius)
  {
    std::vector<cv::DMatch> v_matches; 


    return v_matches;
  }

  std::vector<cv::DMatch> Matcher::GetMatchesByWholeSearch(const Frame& frame0, const Frame& frame1)
  {
    std::vector<cv::DMatch> v_matches; 

    cv::Mat mask;
    m_p_matcher->match(frame0.GetDescriptors(),frame1.GetDescriptors(),v_matches);

    return v_matches;
  }

  std::vector<cv::DMatch> Matcher::GetMatchesByEpipolarSearch(const Frame& frame0, const Frame& frame1)
  {
    std::vector<cv::DMatch> v_matches; 

    return v_matches;
  }
  std::vector<cv::DMatch> Matcher::GetMatchesUsingMotionModel(const Frame& frame0, const Frame& frame1)
  {
    std::vector<cv::DMatch> v_matches; 

    return v_matches;
  }

  void Matcher::ShowMatches(const Frame& frame0, const Frame& frame1,
                            const std::vector<cv::DMatch>& v_matches_01)
  {
    cv::Mat m_output;
    cv::drawMatches(frame0.GetImage(), frame0.GetKeyPoints(),
                    frame1.GetImage(), frame1.GetKeyPoints(),
                    v_matches_01, m_output);

    // std::cout << frame0.GetAssignedKeyPointsNum() << std::endl;
    // std::cout << v_matches_01.size() << std::endl;
    cv::imshow("viewer-matches", m_output);
    cv::waitKey(0);

    return;
  }


} // namespace TS_SfM
