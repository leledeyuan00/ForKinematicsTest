#include "test_kdl/test_kdl.h"

test_kdl::test_kdl()
{
    robot_init();
}

void test_kdl::robot_init()
{
    Q_init_.resize(num_joint_);
    for (size_t i = 0; i < num_joint_; i++)
    {
        robot_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
            robot_frames_[i]));
    }

    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(robot_));
    iksolver_.reset(new KDL::ChainIkSolverPos_LMA(robot_));
  
}

void test_kdl::control_loop()
{
    std::cout << "Start Calculate robot kinematics." << std::endl;
    std::vector<double> fk_out_,ik_out_,ik_in_;
    fk_out_.resize(num_joint_);
    ik_out_.resize(num_joint_);
    ik_in_.resize(num_joint_);
    while (true)
    {
        // FK
        for (size_t i = 0; i < num_joint_; i++)
        {
            float myinput;
            printf ("Enter the position of joint %zu in Degree: ",i);
            scanf ("%f",&myinput);
            Q_init_(i)=(double)myinput * D2R;
        }
        std::cout << "Ik input is: ["; 
        for (size_t i = 0; i < num_joint_; i++)
        {
            std::cout << Q_init_(i) << ", " ;    
        }
        std::cout <<"]" <<std::endl;
        KDL::Frame cartpos;

        int status = fksolver_->JntToCart(Q_init_,cartpos);
        if (status>=0)
        {
            std::cout << cartpos << std::endl;
            memcpy(&fk_out_[0],cartpos.p.data,3*sizeof(double));
            cartpos.M.GetRPY(fk_out_[3],fk_out_[4],fk_out_[5]);
        }
        else{
            std::cout<< "some error"<<std::endl;
            return;
        }
        std::cout << "FK finished!" << std::endl;
        std::cout << "Result is :[" ;
        for (size_t i = 0; i < num_joint_; i++)
        {
            std::cout << fk_out_ [i] * (i>2?R2D:1)<< ", ";
        }
        std::cout<< "]" << std::endl;

        // IK
        std::cout << "Enter the IK input in Degree:" << std::endl;
        for (size_t i = 0; i < num_joint_; i++)
        {
            float myinput;
            std::cout << "Enter the " << pose_name_[i] << std::endl;
            scanf ("%f",&myinput);
            ik_in_[i]=(double)myinput * D2R;
        }
        KDL::Frame cartIn(KDL::Rotation::RPY(ik_in_[3],ik_in_[4],ik_in_[5]),
            KDL::Vector(ik_in_[0],ik_in_[1],ik_in_[2]));
        KDL::JntArray Q_out(num_joint_);

        status = iksolver_->CartToJnt(Q_init_,cartpos,Q_out);
        
        if (status>=0)
        {
            for (size_t i = 0; i < num_joint_; i++)
            {
                ik_out_[i] = Q_out(i);  
            }
        }
        else{
            std::cout<< "some error"<<std::endl;
            return;
        }

        std::cout << "IK finished!" << std::endl;
        std::cout << "Result is :[" ;
        for (size_t i = 0; i < num_joint_; i++)
        {
            std::cout << ik_out_ [i] * R2D<< ", ";
        }
        std::cout<< "]" << std::endl;
    }
    std::cout << "Calculate finished." << std::endl;
}


int main(int argc, char const *argv[])
{
    test_kdl custom_t;
    custom_t.control_loop();
    return 0;
}
