#include <kdl_parser/kdl_parser.hpp>
#include <sys/select.h>
#include <lwr_simulator/lwr_controller.h>

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
LWRController::LWRController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
LWRController::~LWRController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void LWRController::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Get the world name.
  this->world = _parent->GetWorld();

  // Get a pointer to the model
  this->parent_model_ = _parent;

  // Error message if the model couldn't be found
  if (!this->parent_model_)
    gzerr << "Unable to get parent model\n";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&LWRController::UpdateChild, this, _1));
  gzdbg << "plugin model name: " << modelName << "\n";

  // get parameter name
  this->robotPrefix = "";
  if (_sdf->HasElement("robotPrefix"))
    this->robotPrefix = _sdf->GetElement("robotPrefix")->Get<std::string>();
  
  remote_port = 7998;
  if (_sdf->HasElement("remotePort"))
    this->remote_port = _sdf->GetElement("remotePort")->Get<double>();

  remote = "127.0.0.1";
  if (_sdf->HasElement("remoteIP"))
    this->remote = _sdf->GetElement("remoteIP")->Get<std::string>();

  chain_start = std::string("calib_") + this->robotPrefix + "_arm_base_link";
  if (_sdf->HasElement("baseLink"))
    this->chain_start = _sdf->GetElement("baseLink")->Get<std::string>();

  chain_end = this->robotPrefix + "_arm_7_link";
  if (_sdf->HasElement("toolLink"))
    this->chain_end = _sdf->GetElement("toolLink")->Get<std::string>();
    
  gzdbg << "remote : " << remote << " : " << remote_port << "\n";
  
  // get parameter name
  std::string robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
    robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  
  payloadMass_ = 0.0;
  if (_sdf->HasElement("payloadMass"))
    payloadMass_ = _sdf->GetElement("payloadMass")->Get<double>();
  
  
  if (_sdf->HasElement("payloadCOG"))
    payloadCOG_ = _sdf->GetElement("payloadCOG")->Get<math::Vector3>();
  else
    payloadCOG_ = math::Vector3(0,0,0);
    
  if (_sdf->HasElement("gravityDirection"))
    gravityDirection_ = _sdf->GetElement("gravityDirection")->Get<math::Vector3>();
  else
  {
    ROS_WARN("Gravity direction not given to lwrcontroller plugin, using default. \nThis will only work if you robot is standing on the floor !");
    gravityDirection_ = math::Vector3(0,0,-9.81);
  }
  
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  this->rosnode_ = new ros::NodeHandle(robotNamespace);
      
  GetRobotChain();
    
  for(unsigned int i = 0; i< 7; i++)
  {
    // fill in gazebo joints pointer
    std::string joint_name = this->robotPrefix + "_arm_" + (char)(i + 48) + "_joint";
    gazebo::physics::JointPtr joint = this->parent_model_->GetJoint(joint_name);     
    if (joint)
    {
      this->joints_.push_back(joint);
    }
    else
    {
      this->joints_.push_back(gazebo::physics::JointPtr());  // FIXME: cannot be null, must be an empty boost shared pointer
      ROS_ERROR("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
    }
    
    if(_sdf->HasElement(joint_name)) {
      double init = _sdf->GetElement(joint_name)->Get<double>();
      joint->SetAngle(0, init);
    }
    
    // stiffness_(i) = 200.0;
    // damping_(i) = 5.0;
    stiffness_(i) = LWRSIM_DEFAULT_STIFFNESS;
    damping_(i) = LWRSIM_DEFAULT_DAMPING;
    trq_cmd_(i) = LWRSIM_DEFAULT_TRQ_CMD;
    joint_pos_cmd_(i) = joints_[i]->GetAngle(0).Radian();
    
    m_msr_data.data.cmdJntPos[i] = 0.0;
    m_msr_data.data.cmdJntPosFriOffset[i] = 0.0;
    
    //init also stiffness and damping
    m_cmd_data.cmd.jntStiffness[i] = stiffness_(i);
    m_cmd_data.cmd.jntDamping[i] = damping_(i);
    
  }
  
  socketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  setsockopt(socketFd, SOL_SOCKET, SO_REUSEADDR, 0, 0);
  
  bzero((char *) &localAddr, sizeof(localAddr));
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = INADDR_ANY;
  localAddr.sin_port = htons(remote_port + 1);

  if (bind(socketFd, (sockaddr*) &localAddr, sizeof(sockaddr_in)) < 0) {
    ROS_ERROR("Binding of port %d failed ",remote_port);
  }
  else
		ROS_INFO("Bound to port %d",remote_port);
  
  bzero((char *) &remoteAddr, sizeof(remoteAddr));
	remoteAddr.sin_family = AF_INET;
	remoteAddr.sin_addr.s_addr = inet_addr(remote.c_str());
	remoteAddr.sin_port = htons(remote_port);
  
  bzero((char *) &m_msr_data, sizeof(tFriMsrData));
	
  //m_msr_data.robot.control = FRI_CTRL_JNT_IMP;
  m_msr_data.robot.control = FRI_CTRL_POSITION;
  m_msr_data.intf.state = FRI_STATE_MON;
  m_msr_data.robot.power = 0xFFFF;

  m_msr_data.robot.error = 0x0000;
  m_msr_data.robot.warning = 0x0000;
  m_msr_data.intf.desiredCmdSampleTime = 0.001;
  m_msr_data.intf.desiredMsrSampleTime = 0.001;
  
  cnt = 0;
  current_time_ = common::Time::GetWallTime();
}

void LWRController::GetRobotChain()
{
  KDL::Tree my_tree;
  std::string robot_desc_string;
  rosnode_->param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }

  my_tree.getChain(chain_start, chain_end, chain_);
  
  // get last segment
  KDL::Segment *segment_ee_ptr = &(chain_.segments[chain_.getNrOfSegments()-1]);
  // get its dynamic parameters
  KDL::RigidBodyInertia inertia_ee = segment_ee_ptr->getInertia();
  std::string name_ee = segment_ee_ptr->getName();
  double m_ee = inertia_ee.getMass();
  KDL::Vector cog_ee = inertia_ee.getCOG();
  KDL::RotationalInertia rot_inertia_ee = inertia_ee.getRotationalInertia();

  // add payload cog offset and mass
  KDL::Vector payload_cog_offset(payloadCOG_[0], payloadCOG_[1], payloadCOG_[2]);
  m_ee += payloadMass_;
  cog_ee += payload_cog_offset;

  // set the new inertia at the end-effector.
  segment_ee_ptr->setInertia(KDL::RigidBodyInertia(m_ee, cog_ee, rot_inertia_ee)); 

  dyn = new KDL::ChainDynParam(chain_, KDL::Vector(gravityDirection_[0],gravityDirection_[1],gravityDirection_[2]));
  fk = new KDL::ChainFkSolverPos_recursive(chain_);
  jc = new KDL::ChainJntToJacSolver(chain_);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void LWRController::UpdateChild(const common::UpdateInfo &update_info)
{
  struct sockaddr cliAddr;
  unsigned int cliAddr_len;
  KDL::Frame f;
  KDL::Jacobian jac(7);
  KDL::JntSpaceInertiaMatrix H(7);
  KDL::JntArray pos(7);
  KDL::JntArray grav(7);

  previous_time_ = current_time_;
  current_time_ = update_info.realTime;
  common::Time period = current_time_-previous_time_;
  // store the real period
  m_msr_data.intf.desiredMsrSampleTime=period.Double();
  
  for(unsigned int i = 0; i< 7; i++)
  {
    //joint_pos_prev_(i) = joint_pos_(i);
    m_msr_data.data.cmdJntPos[i] = m_msr_data.data.msrJntPos[i] = pos(i) = joint_pos_(i) = joints_[i]->GetAngle(0).Radian();
    // filter is worth less here, as the joint_vel is not transmitted to FRI
    //joint_vel_(i) = (joint_pos_(i) - joint_pos_prev_(i))*(0.2/period.Float()) + joint_vel_(i)*0.8 ;
    joint_vel_(i) = joints_[i]->GetVelocity(0);
  }

  dyn->JntToGravity(pos, grav);
  for(unsigned int i = 0; i< 7; i++)
  {
    m_msr_data.data.gravity[i]=grav(i);
  }

  fk->JntToCart(pos, f);
  m_msr_data.data.msrCartPos[0] = f.M.data[0];
  m_msr_data.data.msrCartPos[1] = f.M.data[1];
  m_msr_data.data.msrCartPos[2] = f.M.data[2];
  m_msr_data.data.msrCartPos[3] = f.p.data[0];

  m_msr_data.data.msrCartPos[4] = f.M.data[3];
  m_msr_data.data.msrCartPos[5] = f.M.data[4];
  m_msr_data.data.msrCartPos[6] = f.M.data[5];
  m_msr_data.data.msrCartPos[7] = f.p.data[1];

  m_msr_data.data.msrCartPos[8] = f.M.data[6];
  m_msr_data.data.msrCartPos[9] = f.M.data[7];
  m_msr_data.data.msrCartPos[10] = f.M.data[8];
  m_msr_data.data.msrCartPos[11] = f.p.data[2];

  jc->JntToJac(pos, jac);
  jac.changeRefFrame(KDL::Frame(f.Inverse().M));
  //Kuka uses Tx, Ty, Tz, Rz, Ry, Rx convention, so we need to swap Rz and Rx
  jac.data.row(3).swap(jac.data.row(5));
  for ( int i = 0; i < FRI_CART_VEC; i++)
    for ( int j = 0; j < LBR_MNJ; j++)
      m_msr_data.data.jacobian[i*LBR_MNJ+j] = jac.data(i,j);

  dyn->JntToMass(pos, H);
  for(unsigned int i=0;i<LBR_MNJ;i++) {
    for(unsigned int j=0;j<LBR_MNJ;j++) {
      m_msr_data.data.massMatrix[LBR_MNJ*i+j] = H.data(i, j);
    }
  }

  if(cnt > 10)
  {
    m_msr_data.intf.state = FRI_STATE_CMD;
  } else 
  {
    m_msr_data.intf.state = FRI_STATE_MON;
  }
  
  
  
  //send msr data
  if (0 > sendto(socketFd, (void*) &m_msr_data, sizeof(m_msr_data), 0,
			(sockaddr*) &remoteAddr, sizeof(remoteAddr))) {
		ROS_ERROR( "Sending datagram failed.");
		//return -1;
	}

  fd_set rd;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1000;
  
  FD_ZERO(&rd);
  FD_SET(socketFd, &rd);
	
  int sret = select(socketFd+1, &rd, NULL, NULL, &tv);
  if(sret > 0) {
    int n = recvfrom(socketFd, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
			(sockaddr*) &cliAddr, &cliAddr_len);
    if (sizeof(tFriCmdData) != n) {
      ROS_DEBUG( "bad packet length : %d should be %zu", n, sizeof(tFriCmdData));
    }

    for(unsigned int i = 0; i < 7; i++) {
      joint_pos_cmd_(i) = m_cmd_data.cmd.jntPos[i];
      
      if( m_msr_data.robot.control == FRI_CTRL_JNT_IMP )
      {
        stiffness_(i) = m_cmd_data.cmd.jntStiffness[i];
        damping_(i) = m_cmd_data.cmd.jntDamping[i];
        trq_cmd_(i) = m_cmd_data.cmd.addJntTrq[i];
      }

      ROS_DEBUG("kuka j%d stiffness %f damping %f",i, stiffness_(i), damping_(i));
    }

    if(cnt < 20) {
      ++cnt;
    }
    
  } else {
    if(cnt > 0)
      --cnt;
  }
  
  trq_ = stiffness_.asDiagonal() * (joint_pos_cmd_ - joint_pos_) - damping_.asDiagonal() * joint_vel_ + trq_cmd_;

  
  for(unsigned int i = 0; i< 7; i++) {
    // add gravity compensation
    joints_[i]->SetForce(0, trq_(i) + grav(i));
  }
  ROS_DEBUG("kuka pos cmd %f pos current %f vel curr %f trq_cmd %f trq %f, grav %f", joint_pos_cmd_(1), joint_pos_(1), joint_vel_(1), trq_cmd_(1), trq_(1), grav(1));
}

GZ_REGISTER_MODEL_PLUGIN(LWRController);

}
