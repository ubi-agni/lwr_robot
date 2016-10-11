/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef LWR_CONTROLLER_HH
#define LWR_CONTROLLER_HH

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "Eigen/Dense"
#include <kuka_lwr_fri/friComm.h>

#include <sys/socket.h> /* for bind socket accept */
#include <unistd.h> /* for close() */
#include <arpa/inet.h>/* for inet_Addr etc*/

#define LWRSIM_DEFAULT_STIFFNESS (200.0)
#define LWRSIM_DEFAULT_CARTSTIFFNESSFORCE (200.0)
#define LWRSIM_DEFAULT_CARTSTIFFNESSTORQUE (30.0)
#define LWRSIM_DEFAULT_CARTDAMPING (0.7)
#define LWRSIM_DEFAULT_DAMPING (1.0)
#define LWRSIM_DEFAULT_TRQ_CMD (0.0)

// OpenKC compatibility
#define OKC_FRI_START 1
#define OKC_FRI_STOP 2
#define OKC_RESET_STATUS 3
#define OKC_SET_CP_STIFFNESS_DAMPING 4
#define OKC_SET_AXIS_STIFFNESS_DAMPING 5
#define OKC_SWITCH_CP_CONTROL 6
#define OKC_SWITCH_AXIS_CONTROL 7
#define OKC_SWITCH_GRAVCOMP 8
#define OKC_SWITCH_POSITION 9
#define OKC_MOVE_START_POSITION 10
#define OKC_RESET_COUNTER 11

#define OROCOS_OKC_DRIVE_OFF 20
#define OROCOS_OKC_DRIVE_ON 21
#define OROCOS_OKC_MOVE_AXIS_POS 22
#define OROCOS_OKC_SWITCH_CONTROL_MODE 30
#define OROCOS_OKC_MOVE_PARK_POS 90
#define OROCOS_OKC_RESET_FRI 98
#define OROCOS_OKC_END_KRL 99

// index starts at 0 in cpp so value is one less than in krl
#define OKC_ACK_IDX 14
#define OKC_SEQ_IDX 13
#define OKC_CMD_IDX 15

#define EPSILON_DET   0.0001

namespace gazebo
{

   class LWRController : public ModelPlugin
   {
      public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// \brief Constructor
      public: LWRController();

      /// \brief Destructor
      public: virtual ~LWRController();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      protected: virtual void UpdateChild(const common::UpdateInfo &update_info);
      
      void GetRobotChain();
      
      private:
      
        bool DriveOnCb(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res);
        bool DriveOffCb(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res);
                              
        void Brake(Eigen::Matrix<double, 7, 1> &pos, KDL::JntArray &grav);
        bool brakes_on_;
                              
        bool isValidRotation(KDL::Rotation &rot);
        /*
         *  \brief pointer to ros node
         */
        ros::NodeHandle* rosnode_;
        ros::ServiceServer drive_on_srv_;
        ros::ServiceServer drive_off_srv_;
        
        std::string model_name_;
        gazebo::physics::ModelPtr parent_model_;
        std::string robotPrefix;
        std::vector<gazebo::physics::JointPtr>  joints_;
        std::string chain_start, chain_end;
        
        double payloadMass_;
        math::Vector3 payloadCOG_;
        math::Vector3 gravityDirection_;
        
         // Pointer to the model
        physics::WorldPtr world;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        
        KDL::Chain chain_;
        KDL::ChainDynParam *dyn;
        KDL::ChainFkSolverPos_recursive *fk;
        KDL::ChainJntToJacSolver *jc;

        std::string base_frame_;

        int cnt;  //!< counter of valid frames
        bool drive_on_; //!< status of the motors (and brakes)
        bool auto_on_; //!< If true (default) the robot switches on after 10 valid datagrams
        
        
        Eigen::Matrix<double, 7, 1> joint_pos_;
        Eigen::Matrix<double, 7, 1> joint_pos_prev_;
        Eigen::Matrix<double, 7, 1> joint_pos_cmd_;
        Eigen::Matrix<double, 7, 1> joint_vel_;
        Eigen::Matrix<double, 7, 1> brake_pos_;
        Eigen::Matrix<double, 7, 1> stiffness_;
        Eigen::Matrix<double, 7, 1> user_stiffness_; // user default
        Eigen::Matrix<double, 7, 1> damping_;
        Eigen::Matrix<double, 7, 1> user_damping_; // user default
        Eigen::Matrix<double, 7, 1> trq_cmd_;
        Eigen::Matrix<double, 7, 1> trq_;
        Eigen::Matrix<double, 7, 7> mass_;
        
        Eigen::Matrix<double, 6, 1> cart_pos_cmd_;
        KDL::Frame T_old_;
        KDL::Frame T_D_;
        Eigen::Matrix<double, 6, 1> ext_tcp_ft_;
        Eigen::Matrix<double, 6, 1> cart_stiffness_;
        Eigen::Matrix<double, 6, 1> user_cart_stiffness_;
        Eigen::Matrix<double, 6, 1> cart_damping_;
        Eigen::Matrix<double, 6, 1> user_cart_damping_;
        
        common::Time previous_time_, current_time_;
        
        int remote_port;
        std::string remote;

        int socketFd;
        struct sockaddr_in localAddr, remoteAddr;

        tFriMsrData m_msr_data;
        tFriCmdData m_cmd_data;
   };

}

#endif

