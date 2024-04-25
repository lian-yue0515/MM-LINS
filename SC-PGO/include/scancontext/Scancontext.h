#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "scancontext/nanoflann.hpp"
#include "scancontext/KDTreeVectorOfVectorsAdaptor.h"
#include "aloam_velodyne/tic_toc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;



void coreImportTest ( void );


// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
public: 
    SCManager( ) = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.
    SCManager(const SCManager& other)
        : polarcontexts_timestamp_(other.polarcontexts_timestamp_),
          polarcontexts_(other.polarcontexts_),
          polarcontext_invkeys_(other.polarcontext_invkeys_),
          polarcontext_vkeys_(other.polarcontext_vkeys_),
          polarcontext_invkeys_mat_(other.polarcontext_invkeys_mat_),
        //   polarcontext_invkeys_to_search_(other.polarcontext_invkeys_to_search_.begin(), other.polarcontext_invkeys_to_search_.end()),
        //   polarcontext_tree_(std::make_unique<InvKeyTree>(*other.polarcontext_tree_)),
          optimal_matchvalue(other.optimal_matchvalue)
          {}
    SCManager(SCManager&& other) noexcept
    {
        polarcontexts_timestamp_ = std::move(other.polarcontexts_timestamp_);
        polarcontexts_ = std::move(other.polarcontexts_);
        polarcontext_invkeys_ = std::move(other.polarcontext_invkeys_);
        polarcontext_vkeys_ = std::move(other.polarcontext_vkeys_);
        polarcontext_invkeys_mat_ = std::move(other.polarcontext_invkeys_mat_);
        // polarcontext_invkeys_to_search_ = std::move(other.polarcontext_invkeys_to_search_);
        // polarcontext_tree_ = std::move(other.polarcontext_tree_); 
        optimal_matchvalue = other.optimal_matchvalue;
        other.polarcontexts_timestamp_.clear();
        other.polarcontexts_.clear();
        other.polarcontext_invkeys_.clear();
        other.polarcontext_vkeys_.clear();
        other.polarcontext_invkeys_mat_.clear();
        // other.polarcontext_invkeys_to_search_.clear();
        // other.polarcontext_tree_ = nullptr;
        other.optimal_matchvalue = 0.0;
    }
    SCManager& operator=(const SCManager& other) {
        if (this == &other) {
            return *this;
        }

        polarcontexts_timestamp_ = other.polarcontexts_timestamp_;
        polarcontexts_ = other.polarcontexts_;
        polarcontext_invkeys_ = other.polarcontext_invkeys_;
        polarcontext_vkeys_ = other.polarcontext_vkeys_;
        polarcontext_invkeys_mat_ = other.polarcontext_invkeys_mat_;
        // polarcontext_invkeys_to_search_ = other.polarcontext_invkeys_to_search_;
        // polarcontext_tree_ = std::make_unique<InvKeyTree>(*other.polarcontext_tree_);
        optimal_matchvalue = other.optimal_matchvalue;

        return *this;
    }
    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void ); // int: nearest node index, float: relative yaw

    std::pair<int, float> similar_detectLoopClosureID( KeyMat Curr_Key, std::vector<Eigen::MatrixXd>Curr_polarcontexts ); // int: nearest node index, float: relative yaw 

    // for ltslam 
    // User-side API for multi-session
    void saveScancontextAndKeys( Eigen::MatrixXd _scd );
    std::pair<int, float> detectLoopClosureIDBetweenSession ( std::vector<float>& curr_key,  Eigen::MatrixXd& curr_desc);

    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    const double LIDAR_HEIGHT = 2.0; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    const int    PC_NUM_RING = 20; // 20 in the original paper (IROS 18)
    const int    PC_NUM_SECTOR = 60; // 60 in the original paper (IROS 18)
    double PC_MAX_RADIUS = 20.0; // 80 meter max in the original paper (IROS 18)
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    const int    NUM_EXCLUDE_RECENT = 30; 
    const int    NUM_CANDIDATES_FROM_TREE = 3; // 10 is enough. (refer the IROS 18 paper) 

    // loop thres
    const double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)

    double SC_DIST_THRES = 0.6; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    // const double SC_DIST_THRES = 0.7; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    const int    TREE_MAKING_PERIOD_ = 5; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration, it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec. But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
    int          tree_making_period_conter = 0;

    // setter
    void setSCdistThres(double _new_thres);
    void setMaximumRadius(double _max_r);

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.  
    std::vector<Eigen::MatrixXd> polarcontexts_;  
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;    
    std::unique_ptr<InvKeyTree> polarcontext_tree_;    


    double optimal_matchvalue;

}; 

