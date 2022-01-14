#pragma once
// std
#include <memory>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>

// kdl
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

const size_t num_joint_ = 6;
const static double PI = 3.141592657;
const static double D2R = PI/180;
const static double R2D = 180/PI;

const static std::vector<KDL::Frame> robot_frames_ {
    KDL::Frame::DH( 0.0, -PI/2, 0.3991,  0), // link1
    KDL::Frame::DH( -0.448, 0   , 0.0,    PI/2), // link2
    KDL::Frame::DH( -0.042, PI/2 , 0,  0), // link3
    KDL::Frame::DH( 0.0, -PI/2, 0.451,    0), // link4
    KDL::Frame::DH( 0.0, PI/2, 0,  0), // link5
    KDL::Frame::DH( 0.0, 0 , 0.082,    0), // link6
};

const static std::vector<std::string> pose_name_ {"x","y","z","R","P","Y"};

class test_kdl
{
public:
    test_kdl();
    void control_loop();
private:
    void robot_init();

    KDL::Chain robot_;
    KDL::JntArray Q_init_;
    
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> iksolver_;
};

