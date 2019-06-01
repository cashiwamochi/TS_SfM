#include "Matcher.h"


namespace TS_SfM {
  
  Matcher::Matcher(const CheckType& _checktype, const SearchType& _searchtype)
    : m_checktype(_checktype), m_searchtype(_searchtype)
  {

    if(m_checktype == CrossCheck) {
      m_p_matcher.reset(new cv::BFMatcher(cv::NORM_L1, true));
    }
    else {
      m_p_matcher.reset(new cv::BFMatcher(cv::NORM_L1, false));
    }
   
  }


  template<typename T> 
  std::vector<cv::DMatch>
    Matcher::GetMatches(const Frame& frame0, const Frame& frame1, T value) 
    {
      std::vector<cv::DMatch> v_matches;
      switch(m_searchtype) {
        case Radius:
          v_matches = GetMatchesByRadiusSearch(frame0, frame1, value);
          break;
        case Grid:

          break;
        case Whole:

          break;
        default:
          break;
      } 


      return v_matches;
    }

  std::vector<cv::DMatch> GetMatchesByGridSearch(const Frame& frame0, const Frame& frame1, int neighbor)
  {
    std::vector<cv::DMatch> v_matches; 


    return v_matches;
  }

  std::vector<cv::DMatch> GetMatchesByRadiusSearch(const Frame& frame0, const Frame& frame1, int radius)
  {
    std::vector<cv::DMatch> v_matches; 


    return v_matches;
  }

  std::vector<cv::DMatch> GetMatchesByWholeSearch(const Frame& frame0, const Frame& frame1)
  {
    std::vector<cv::DMatch> v_matches; 


    return v_matches;
  }

} // namespace TS_SfM
