#include <kdl_parser/kdl_parser.hpp>
#include <sys/select.h>
#include <errno.h>
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
  model_name_ = _sdf->GetParent()->Get<std::string>("name");

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
  gzdbg << "plugin model name: " << model_name_ << "\n";

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

  chain_start = this->robotPrefix + "_arm_base_link";
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

  gzdbg << "payloadCOG : " << payloadCOG_[0] << ", " << payloadCOG_[1] << ", " << payloadCOG_[2] << "\n";
  gzdbg << "payload mass : " << payloadMass_ << "\n";

  if (_sdf->HasElement("gravityDirection"))
    gravityDirection_ = _sdf->GetElement("gravityDirection")->Get<math::Vector3>();
  else
  {
    ROS_WARN_STREAM("Gravity direction not given to lwrcontroller plugin " << model_name_ << ", using default. \nThis will only work if you robot is standing on the floor !");
    gravityDirection_ = math::Vector3(0,0,-9.81);
  }
  gzdbg << "gravity Dir : " << gravityDirection_[0] << ", " << gravityDirection_[1] << ", " << gravityDirection_[2] << "\n";
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  if (_sdf->HasElement("auto_on"))
  {
    auto_on_ = _sdf->GetElement("auto_on")->Get<bool>();
    gzdbg << "auto_on : " << auto_on_ << "\n";
    if (auto_on_)
      ROS_INFO_STREAM("lwr_ctrl " << model_name_ << ":started in auto_on mode (user request)");
    else
      ROS_INFO_STREAM("lwr_ctrl " << model_name_ << ":started without auto_on mode (user request)");
  }
  else
  {
    ROS_INFO_STREAM("lwr_ctrl " << model_name_ << ":started in auto_on mode (default)");
    gzdbg << "auto_on: default to true" << "\n";
    auto_on_ = true;
  }

  this->rosnode_ = new ros::NodeHandle(robotNamespace);
  GetRobotChain();

  for(unsigned int i = 0; i< LBR_MNJ; i++)
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
      ROS_ERROR_STREAM("lwr_ctrl " << model_name_ << ": A joint named \"" << joint_name << "\" is not part of Mechanism Controlled joints.\n");
    }

    if(_sdf->HasElement(joint_name + "_init")) {
      double init = _sdf->GetElement(joint_name + "_init")->Get<double>();
      joint->SetPosition(0, init);
      freeze_pos_(i) = init; // also lock brakes at init pos
    }
    
    if (_sdf->HasElement(joint_name + "_igain"))
    {
      i_gain_(i) = _sdf->GetElement(joint_name + "_igain")->Get<double>();
      gzdbg << "joint " << joint_name << " using igain : " << i_gain_(i) << "\n";
    }
    else
      i_gain_(i) = LWRSIM_DEFAULT_IGAIN;

    // stiffness_(i) = 200.0;
    // damping_(i) = 5.0;
    
    stiffness_(i) = LWRSIM_DEFAULT_STIFFNESS;
    user_stiffness_(i) = LWRSIM_DEFAULT_STIFFNESS;
    damping_(i) = LWRSIM_DEFAULT_DAMPING;
    user_damping_(i) = LWRSIM_DEFAULT_DAMPING;
    i_term_(i) = 0.0;

    trq_cmd_(i) = LWRSIM_DEFAULT_TRQ_CMD;
    joint_pos_cmd_(i) = joints_[i]->GetAngle(0).Radian();

    m_msr_data.data.cmdJntPos[i] = 0.0;
    m_msr_data.data.cmdJntPosFriOffset[i] = 0.0;

    //init also stiffness and damping
    m_cmd_data.cmd.jntStiffness[i] = stiffness_(i);
    m_cmd_data.cmd.jntDamping[i] = damping_(i);
  }

  i_max_ = LWRSIM_DEFAULT_IMAX;

  for(unsigned int i = 0; i< FRI_CART_VEC; i++)
  {
    cart_pos_cmd_(i) = 0.0;
    ext_tcp_ft_(i) = 0.0;
    cart_damping_(i) = LWRSIM_DEFAULT_CARTDAMPING;
    user_cart_damping_(i) = LWRSIM_DEFAULT_CARTDAMPING;
    if ( i < 3)
    {
      cart_stiffness_(i) = LWRSIM_DEFAULT_CARTSTIFFNESSFORCE;
      user_cart_stiffness_(i) = LWRSIM_DEFAULT_CARTSTIFFNESSFORCE;
    }
    else
    {
      cart_stiffness_(i) = LWRSIM_DEFAULT_CARTSTIFFNESSTORQUE;
      user_cart_stiffness_(i) = LWRSIM_DEFAULT_CARTSTIFFNESSTORQUE;
    }
  }

  gzdbg << "Initialized joints" << "\n";

  socketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  setsockopt(socketFd, SOL_SOCKET, SO_REUSEADDR, 0, 0);

  bzero((char *) &localAddr, sizeof(localAddr));
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = INADDR_ANY;
  localAddr.sin_port = htons(remote_port + 1);

  if (bind(socketFd, (sockaddr*) &localAddr, sizeof(sockaddr_in)) < 0) {
    ROS_ERROR_STREAM("lwr_ctrl " << model_name_ << ":Binding of port " << remote_port << " failed");
  }
  else
		ROS_INFO_STREAM("lwr_ctrl " << model_name_ << ":Bound to port " <<remote_port);

  bzero((char *) &remoteAddr, sizeof(remoteAddr));
	remoteAddr.sin_family = AF_INET;
	remoteAddr.sin_addr.s_addr = inet_addr(remote.c_str());
	remoteAddr.sin_port = htons(remote_port);

  bzero((char *) &m_msr_data, sizeof(tFriMsrData));

  //m_msr_data.robot.control = FRI_CTRL_JNT_IMP;
  m_msr_data.robot.control = FRI_CTRL_POSITION;
  m_msr_data.intf.state = FRI_STATE_MON;
  m_msr_data.robot.power = 0x0000;

  m_msr_data.robot.error = 0x0000;
  m_msr_data.robot.warning = 0x0000;
  m_msr_data.intf.desiredCmdSampleTime = 0.001;
  m_msr_data.intf.desiredMsrSampleTime = 0.001;

  cnt = 0;
  drive_on_ = false;
  brakes_on_ = true; // initially true to lock brakes at init
  freeze_ = true;

  // services for switching on and off the drives and release or activate the brakes
  drive_on_srv_ = rosnode_->advertiseService("drive_on", &LWRController::DriveOnCb, this);
  drive_off_srv_ = rosnode_->advertiseService("drive_off", &LWRController::DriveOffCb, this);

  current_time_ = common::Time::GetWallTime();
}

void LWRController::GetRobotChain()
{
  KDL::Tree my_tree;
  std::string robot_desc_string;
  rosnode_->param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR_STREAM("lwr_ctrl " << model_name_ << ":Failed to construct kdl tree");
  }

  my_tree.getChain(chain_start, chain_end, chain_);

  ROS_ASSERT_MSG(chain_.getNrOfSegments()>0,"Chain has no elements");
  // get last segment
  KDL::Segment *segment_ee_ptr = &(chain_.segments[chain_.getNrOfSegments()-1]);
  // get its dynamic parameters
  KDL::RigidBodyInertia inertia_ee = segment_ee_ptr->getInertia();
  // std::string name_ee = segment_ee_ptr->getName();
  // retrieve the current mass, cog and inertia of the last segment
  double m_ee = inertia_ee.getMass();
  KDL::Vector cog_ee = inertia_ee.getCOG();
  gzdbg << "initial CoG of ee : " << cog_ee[0] << ", " << cog_ee[1] << ", " << cog_ee[2] << "\n";
  gzdbg << "initial mass of ee : " << m_ee << "\n";
  KDL::RotationalInertia rot_inertia_ee = inertia_ee.getRotationalInertia();

  // add payload cog offset and mass
  // retrieve tool mass and cog offset. 
  // the offset is the relative position of the tool com from the last segment cog
  // given in the frame orientation of the last segment.
  // so this offset is valid only for an certain assembly (calib of the tool)
  KDL::Vector payload_cog_offset(payloadCOG_[0], payloadCOG_[1], payloadCOG_[2]);
  double m_combined = m_ee + payloadMass_;
  KDL::Vector cog_offset_combined = (payload_cog_offset - cog_ee) * payloadMass_ / m_combined;
  cog_ee += cog_offset_combined;
  
  gzdbg << "combined CoG of ee : " << cog_ee[0] << ", " << cog_ee[1] << ", " << cog_ee[2] << "\n";
  gzdbg << "combined mass of ee : " << m_combined << "\n";

  // set the new inertia at the end-effector.
  segment_ee_ptr->setInertia(KDL::RigidBodyInertia(m_combined, cog_ee, rot_inertia_ee));

  dyn = new KDL::ChainDynParam(chain_, KDL::Vector(gravityDirection_[0],gravityDirection_[1],gravityDirection_[2]));
  fk = new KDL::ChainFkSolverPos_recursive(chain_);
  jc = new KDL::ChainJntToJacSolver(chain_);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void LWRController::UpdateChild(const common::UpdateInfo &update_info)
{
  struct sockaddr cliAddr;
  unsigned int cliAddr_len = sizeof(cliAddr);
  KDL::Frame T;
  KDL::Jacobian jac(LBR_MNJ);
  KDL::JntSpaceInertiaMatrix H(LBR_MNJ);
  KDL::JntArray pos(LBR_MNJ);
  KDL::JntArray grav(LBR_MNJ);

  previous_time_ = current_time_;
  current_time_ = update_info.realTime;
  common::Time period = current_time_-previous_time_;
  // store the real period
  m_msr_data.intf.desiredMsrSampleTime=period.Double();

  for(unsigned int i = 0; i< LBR_MNJ; i++)
  {
    //joint_pos_prev_(i) = joint_pos_(i);
    m_msr_data.data.cmdJntPos[i] = m_msr_data.data.msrJntPos[i] = pos(i) = joint_pos_(i) = joints_[i]->GetAngle(0).Radian();

    // filter is worth less here, as the joint_vel is not transmitted to FRI
    //joint_vel_(i) = (joint_pos_(i) - joint_pos_prev_(i))*(0.2/period.Float()) + joint_vel_(i)*0.8 ;
    joint_vel_(i) = joints_[i]->GetVelocity(0);
    // reset torque;
    trq_(i) = 0;
  }

  dyn->JntToGravity(pos, grav);
  for(unsigned int i = 0; i< LBR_MNJ; i++)
  {
    m_msr_data.data.gravity[i]=grav(i);
  }

  fk->JntToCart(pos, T);
  m_msr_data.data.msrCartPos[0] = T.M.data[0];
  m_msr_data.data.msrCartPos[1] = T.M.data[1];
  m_msr_data.data.msrCartPos[2] = T.M.data[2];
  m_msr_data.data.msrCartPos[3] = T.p.data[0];

  m_msr_data.data.msrCartPos[4] = T.M.data[3];
  m_msr_data.data.msrCartPos[5] = T.M.data[4];
  m_msr_data.data.msrCartPos[6] = T.M.data[5];
  m_msr_data.data.msrCartPos[7] = T.p.data[1];

  m_msr_data.data.msrCartPos[8] = T.M.data[6];
  m_msr_data.data.msrCartPos[9] = T.M.data[7];
  m_msr_data.data.msrCartPos[10] = T.M.data[8];
  m_msr_data.data.msrCartPos[11] = T.p.data[2];

  jc->JntToJac(pos, jac);
  KDL::Jacobian jac_fri = jac;
  jac_fri.changeRefFrame(KDL::Frame(T.Inverse().M));
  //Kuka uses Tx, Ty, Tz, Rz, Ry, Rx convention, so we need to swap Rz and Rx
  jac_fri.data.row(3).swap(jac_fri.data.row(5));
  for ( int i = 0; i < FRI_CART_VEC; i++)
    for ( int j = 0; j < LBR_MNJ; j++)
      m_msr_data.data.jacobian[i*LBR_MNJ+j] = jac_fri.data(i,j);

  dyn->JntToMass(pos, H);
  for(unsigned int i=0;i<LBR_MNJ;i++) {
    for(unsigned int j=0;j<LBR_MNJ;j++) {
      m_msr_data.data.massMatrix[LBR_MNJ*i+j] = H.data(i, j);
      mass_(i,j) = H.data(i, j);
    }
  }

  if (cnt <= 10)
  {
    m_msr_data.intf.quality=FRI_QUALITY_BAD;
    if (m_msr_data.intf.state != FRI_STATE_MON)
    {
      ROS_INFO_STREAM("lwr_ctrl " << model_name_ << ":bad communication, changing to FRI_STATE_MON");
      m_msr_data.intf.state = FRI_STATE_MON;
    }
    drive_on_ = false;
    brakes_on_ = true;
    m_msr_data.robot.power = 0x0;
  }
  else
  {
    if (cnt <= 15)
      m_msr_data.intf.quality=FRI_QUALITY_OK;
    else
      m_msr_data.intf.quality=FRI_QUALITY_PERFECT;
    if (auto_on_)
    {
      drive_on_ = true;

      // don't release brakes here, only if valid data
      if (m_msr_data.intf.state == FRI_STATE_MON)
      {
        ROS_INFO_STREAM("lwr_ctrl " << model_name_ << ":communication ok, changing to FRI_STATE_CMD");
        m_msr_data.intf.state = FRI_STATE_CMD;
      }
    }
  }

  if (drive_on_ && !brakes_on_)
    m_msr_data.robot.power = 0xFFFF;
  else
  {
    m_msr_data.robot.power = 0x0;
    if (!drive_on_)
      m_msr_data.intf.state = FRI_STATE_MON;
  }

  // send msr data to socket
  if (0 > sendto(socketFd, (void*) &m_msr_data, sizeof(m_msr_data), 0,
			(sockaddr*) &remoteAddr, sizeof(remoteAddr))) {
		ROS_ERROR_STREAM("lwr_ctrl " << model_name_ << ":Sending datagram failed.");
		//return -1;
	}

  fd_set rd;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 333;

  FD_ZERO(&rd);
  FD_SET(socketFd, &rd);
  
  bool cart_initialized = false;

  // receive cmd data from socket
  int sret = select(socketFd+1, &rd, NULL, NULL, &tv);
  if(sret > 0) {
    int n = recvfrom(socketFd, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
			(sockaddr*) &cliAddr, &cliAddr_len);
    if (sizeof(tFriCmdData) != n) {
      if (n == -1)
        ROS_DEBUG_STREAM_THROTTLE_NAMED(0.5, "udp", "lwr_ctrl " << model_name_ << ":recv err : " << strerror(errno));
      else
        ROS_DEBUG_STREAM_THROTTLE_NAMED(0.5, "udp", "lwr_ctrl " << model_name_ << ":bad packet length : " << n << " should be " << sizeof(tFriCmdData));
      brakes_on_ = true;
      drive_on_ = false;
      Freeze(joint_pos_, grav);
    }
    else
    {
      // process KRL CMD
      bool ctrl_mode_switched = false;
      {
        // unpack KRL CMD
        if ((m_cmd_data.krl.boolData & (1 << 0))) {
          ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":Received a KRL command");
          // copy cmd to msr
          for(unsigned int i=0 ; i < FRI_USER_SIZE ; ++i) {
            m_msr_data.krl.intData[i] = m_cmd_data.krl.intData[i];
            m_msr_data.krl.realData[i] = 0.0;//m_cmd_data.krl.realData[i];
          }

          // switch fri state (see OpenKC commands)
          switch(m_cmd_data.krl.intData[OKC_CMD_IDX])
          {
            case OKC_FRI_START:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":Received a FRISTART cmd");
              if (drive_on_)
              {
                if(m_msr_data.intf.state != FRI_STATE_CMD)
                {
                  m_msr_data.intf.state = FRI_STATE_CMD;
                  ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":switched to FRISTATE_CMD");
                }
              }
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_FRI_START;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 100;
              ctrl_mode_switched = true;
              break;

            case OKC_FRI_STOP:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":switched to FRISTATE_MON");
              brakes_on_ = false;
              if(m_msr_data.intf.state != FRI_STATE_MON)
              {
                m_msr_data.intf.state = FRI_STATE_MON;
              }
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_FRI_STOP;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 101;
              ctrl_mode_switched = true;
              break;
              
            case OKC_RESET_STATUS:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":reset counters");
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_RESET_STATUS;
              m_msr_data.krl.intData[OKC_SEQ_IDX] = 0;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 111;
              ctrl_mode_switched = true;
              break;

            case OKC_SWITCH_CP_CONTROL:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":switched to OKC_SWITCH_CP_CONTROL");
              m_msr_data.robot.control = FRI_CTRL_CART_IMP;
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_SWITCH_CP_CONTROL;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 104;


              //initialize desired pose to current pose
              m_cmd_data.cmd.cartPos[0] = T.M.data[0];
              m_cmd_data.cmd.cartPos[1] = T.M.data[1];
              m_cmd_data.cmd.cartPos[2] = T.M.data[2];
              m_cmd_data.cmd.cartPos[3] = T.p.data[0];

              m_cmd_data.cmd.cartPos[4] = T.M.data[3];
              m_cmd_data.cmd.cartPos[5] = T.M.data[4];
              m_cmd_data.cmd.cartPos[6] = T.M.data[5];
              m_cmd_data.cmd.cartPos[7] = T.p.data[1];

              m_cmd_data.cmd.cartPos[8] = T.M.data[6];
              m_cmd_data.cmd.cartPos[9] = T.M.data[7];
              m_cmd_data.cmd.cartPos[10] = T.M.data[8];
              m_cmd_data.cmd.cartPos[11] = T.p.data[2];
              T_old_ = T;
              cart_initialized = true;
              ctrl_mode_switched = true;
              break;

            case OKC_SWITCH_AXIS_CONTROL:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":switched to OKC_SWITCH_AXIS_CONTROL");
              m_msr_data.robot.control = FRI_CTRL_JNT_IMP;
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_SWITCH_AXIS_CONTROL;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 105;
              ctrl_mode_switched = true;
              break;

            case OKC_SWITCH_POSITION:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":switched to OKC_SWITCH_POSITION");
              m_msr_data.robot.control = FRI_CTRL_POSITION;
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_SWITCH_POSITION;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 107;
              ctrl_mode_switched = true;
              break;

            case OKC_SWITCH_GRAVCOMP:
              // not implemented, unknown mode in the real robot, what does it do ?
              //m_msr_data.robot.control = FRI_CTRL_JNT_IMP; 
              //gc_only_mode_ = true;
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_SWITCH_GRAVCOMP;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 106;
              ctrl_mode_switched = true;
              break;

            case OROCOS_OKC_SWITCH_CONTROL_MODE:
              ROS_DEBUG_STREAM_NAMED("krl","lwr_ctrl " << model_name_ << ":switched to OROCOS_OKC_SWITCH_CONTROL_MODE");
              switch(m_cmd_data.krl.intData[1])
              {
                case 10:
                  m_msr_data.robot.control = FRI_CTRL_POSITION;
                  break;
                case 20:
                  m_msr_data.robot.control = FRI_CTRL_CART_IMP;
                  break;
                case 30:
                  m_msr_data.robot.control = FRI_CTRL_JNT_IMP;
                  break;
                default:
                  m_msr_data.robot.control = FRI_CTRL_OTHER;
                  break;
              }
              //m_msr_data.krl.intData[OKC_ACK_IDX] = OROCOS_OKC_SWITCH_CONTROL_MODE;
              //m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              //m_msr_data.krl.intData[OKC_CMD_IDX] = 107;
              ctrl_mode_switched = true;
              break;
              
            case OKC_SET_AXIS_STIFFNESS_DAMPING:
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_SET_AXIS_STIFFNESS_DAMPING;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 103;
              
              for (unsigned int i = 0; i < LBR_MNJ; i++)
              {
                user_stiffness_(i) = m_cmd_data.krl.intData[8+i];
                user_damping_(i) = m_cmd_data.krl.realData[9+i];
              }
              ctrl_mode_switched = true;
              break;
              
            case OKC_SET_CP_STIFFNESS_DAMPING:
              m_msr_data.krl.intData[OKC_ACK_IDX] = OKC_SET_CP_STIFFNESS_DAMPING;
              m_msr_data.krl.intData[OKC_SEQ_IDX]++;
              m_msr_data.krl.intData[OKC_CMD_IDX] = 102;
              
              for (unsigned int i = 0; i < 6; i++)
              {
                user_cart_stiffness_(i) = m_cmd_data.krl.intData[9+i];
                user_cart_damping_(i) = m_cmd_data.krl.realData[10+i];
              }
              ctrl_mode_switched = true;
              break;

            default:
              ctrl_mode_switched = true;
              break;
          }
        }

        if(ctrl_mode_switched)
        {
          ROS_DEBUG_STREAM_NAMED("krl", "lwr_ctrl " << model_name_ << ":kuka ctrl mode switched");
          ctrl_mode_switched = false;
          m_msr_data.krl.boolData |= (1 << 0);
        }
        else
        {
          m_msr_data.krl.boolData &= ~(1 << 0);
        }

        if(drive_on_)
        {
          // do the control
          // Joint control modes
          if ( m_msr_data.robot.control == FRI_CTRL_JNT_IMP ||  m_msr_data.robot.control == FRI_CTRL_POSITION ) {
            if (m_msr_data.intf.state == FRI_STATE_CMD)
            {
              brakes_on_ = false;
              freeze_ = false;
              for(unsigned int i = 0; i < LBR_MNJ; i++) {
                joint_pos_cmd_(i) = m_cmd_data.cmd.jntPos[i];

                // Joint Position (high impendance)
                if( m_msr_data.robot.control == FRI_CTRL_POSITION ) {
                  stiffness_(i) = LWRSIM_DEFAULT_STIFFNESS;
                  damping_(i) = LWRSIM_DEFAULT_DAMPING;
                  trq_cmd_(i) = LWRSIM_DEFAULT_TRQ_CMD;

                  //  compute the iterm
                  double delta_t = 0.001; // (assume 1 ms loop)
                  i_term_(i) = i_term_(i) + (joint_pos_cmd_(i) - joint_pos_(i)) * delta_t;
                  if (i_term_(i) > i_max_)
                    i_term_(i) = i_max_;
                  if (i_term_(i) < -i_max_)
                    i_term_(i) = -i_max_;
                }

                // Joint Impedance mode
                if( m_msr_data.robot.control == FRI_CTRL_JNT_IMP ) {
                  // if stiffness given
                  if (m_cmd_data.cmd.cmdFlags & FRI_CMD_JNTSTIFF)
                  {
                    stiffness_(i) = m_cmd_data.cmd.jntStiffness[i];
                  }
                  else // use user stiffness instead
                  {
                    stiffness_(i) = user_stiffness_(i);
                  }

                  // if damping given
                  if (m_cmd_data.cmd.cmdFlags & FRI_CMD_JNTDAMP)
                  {
                    damping_(i) = m_cmd_data.cmd.jntDamping[i];
                  }
                  else // use user damping instead
                  {
                    damping_(i) = user_damping_(i);
                  }
                  
                  // if trq given
                  if (m_cmd_data.cmd.cmdFlags & FRI_CMD_JNTTRQ)
                  {
                    trq_cmd_(i) = m_cmd_data.cmd.addJntTrq[i];
                  }
                  else
                  {
                    trq_cmd_(i) = LWRSIM_DEFAULT_TRQ_CMD;
                  }
                  
                  // iterm must be zero otherwise one cannot get impendance
                  i_term_(i) = 0.0;
                }
                ROS_DEBUG_THROTTLE_NAMED(0.1, "joint", "lwr_ctrl %s:kuka j%d stiffness %f damping %f", model_name_.c_str(), i, stiffness_(i), damping_(i));
              }

              // compute the torque
              trq_ = stiffness_.asDiagonal() * (joint_pos_cmd_ - joint_pos_) - damping_.asDiagonal() * joint_vel_ + i_gain_.asDiagonal() * i_term_ + trq_cmd_;
              
              ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1, "joint", "i_term(0) " << i_term_(0) << " i_gain(0) " << i_gain_(0) << " trq(0) " << trq_(0));

              // add gravity compensation
              for(unsigned int i = 0; i< LBR_MNJ; i++) {
                joints_[i]->SetForce(0, trq_(i) + grav(i));
              }
              ROS_DEBUG_STREAM_THROTTLE_NAMED(5.0, "krl", "lwr_ctrl " << model_name_ << ":joint control");
            }
            else
            {
              Freeze(joint_pos_, grav);
            }
          }
          else
          {
            // Cart control mode
            if ( m_msr_data.robot.control == FRI_CTRL_CART_IMP ) {
              if (m_msr_data.intf.state == FRI_STATE_CMD) {
                brakes_on_ = false;
                freeze_ = false;
                joint_pos_cmd_ = joint_pos_; //maybe take the joint pos from previous cycle
                // commanded cart pos

                
                if (m_cmd_data.cmd.cmdFlags & FRI_CMD_CARTPOS | cart_initialized)
                {
                  T_D_.M = KDL::Rotation(m_cmd_data.cmd.cartPos[0],
                      m_cmd_data.cmd.cartPos[1], m_cmd_data.cmd.cartPos[2],
                      m_cmd_data.cmd.cartPos[4], m_cmd_data.cmd.cartPos[5],
                      m_cmd_data.cmd.cartPos[6], m_cmd_data.cmd.cartPos[8],
                      m_cmd_data.cmd.cartPos[9], m_cmd_data.cmd.cartPos[10]);
                  T_D_.p.x(m_cmd_data.cmd.cartPos[3]);
                  T_D_.p.y(m_cmd_data.cmd.cartPos[7]);
                  T_D_.p.z(m_cmd_data.cmd.cartPos[11]);
                  ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka TD %f, %f , %f, \n%f, %f, %f,\n %f, %f, %f,\n p %f, %f, %f", model_name_.c_str(), T_D_.M.data[0],
                                        T_D_.M.data[1], T_D_.M.data[2],
                                        T_D_.M.data[3], T_D_.M.data[4],
                                        T_D_.M.data[5], T_D_.M.data[6],
                                        T_D_.M.data[7], T_D_.M.data[8],
                                        T_D_.p.x(), T_D_.p.y() , T_D_.p.z());
                }
                

                // validate the desired pose
                if (!isValidRotation(T_D_.M))
                {
                    ROS_DEBUG_STREAM_THROTTLE_NAMED(0.1,"cart","lwr_ctrl " << model_name_ << ": INVALID Desired Pose");
                    T_D_.M = KDL::Rotation::Identity();
                }

                // twist
                ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka T %f, %f , %f, \n%f, %f, %f,\n %f, %f, %f,\n p %f, %f, %f", model_name_.c_str(), T.M.data[0],
                                        T.M.data[1], T.M.data[2],
                                        T.M.data[3], T.M.data[4],
                                        T.M.data[5], T.M.data[6],
                                        T.M.data[7], T.M.data[8],
                                        T.p.x(), T.p.y() , T.p.z());
                ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka T_old %f, %f , %f, \n%f, %f, %f,\n %f, %f, %f,\n p %f, %f, %f", model_name_.c_str(), T_old_.M.data[0],
                                        T_old_.M.data[1], T_old_.M.data[2],
                                        T_old_.M.data[3], T_old_.M.data[4],
                                        T_old_.M.data[5], T_old_.M.data[6],
                                        T_old_.M.data[7], T_old_.M.data[8],
                                        T_old_.p.x(), T_old_.p.y() , T_old_.p.z());
                KDL::Twist cart_twist;// = KDL::diff(T_old_, T, m_msr_data.intf.desiredMsrSampleTime);
                cart_twist = T.M.Inverse() * cart_twist;
                T_old_ = T;

                ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka cart_twist vel %f, %f , %f, \nrot %f, %f, %f,", model_name_.c_str(), cart_twist.vel.data[0],
                  cart_twist.vel.data[1], cart_twist.vel.data[2],
                  cart_twist.rot.data[0], cart_twist.rot.data[1],
                  cart_twist.rot.data[2] );

                // ext tool wrench, cart stiffness and damping
                for(unsigned int i = 0; i < FRI_CART_VEC; i++) {
                  if (m_cmd_data.cmd.cmdFlags & FRI_CMD_TCPFT)
                    ext_tcp_ft_(i) = m_cmd_data.cmd.addTcpFT[i];
                  if (m_cmd_data.cmd.cmdFlags & (FRI_CMD_CARTSTIFF | FRI_CMD_CARTDAMP))
                  {
                    cart_stiffness_(i) = m_cmd_data.cmd.cartStiffness[i];
                    cart_damping_(i) = m_cmd_data.cmd.cartDamping[i];
                  }
                  else
                  {
                    cart_stiffness_(i) = user_cart_stiffness_(i);
                    cart_damping_(i) = user_cart_damping_(i);
                  }
                }
                ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka cart_stiffness_  %f, %f, %f, %f, %f, %f", model_name_.c_str(),  cart_stiffness_(0),
                  cart_stiffness_(1),cart_stiffness_(2),cart_stiffness_(3),cart_stiffness_(4),cart_stiffness_(5));
                ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka cart_damping_  %f, %f, %f, %f, %f, %f", model_name_.c_str(), cart_damping_(0),
                  cart_damping_(1),cart_damping_(2),cart_damping_(3),cart_damping_(4),cart_damping_(5));

                // Start of user code ImpedanceControl
                // Code from lwr_impedance_controller RCPRG
                // T is current pose, TD is desired  TC is tool, TS is spring
                KDL::Frame T_C, T_S;
                Eigen::Matrix<double, 7, 7> Kj;
                Eigen::Matrix<double, 6, 1> K0;
                Eigen::Matrix<double, 7, 7> Mi, N;
                Eigen::Matrix<double, 6, 1> F;
                Eigen::Matrix<double, 7, 6> Ji, jT;
                Eigen::Matrix<double, 7, 1> tau;
                Eigen::Matrix<double, 4, 1> e;
                Eigen::Matrix<double, 6, 6> A, A1, Kc1, Dc, Q;

                Eigen::GeneralizedSelfAdjointEigenSolver< Eigen::Matrix<double, 6, 6> > es;

                //if (port_Tool.read(tool_pos) == RTT::NewData) {
                //  tf::PoseMsgToKDL(tool_pos, T_T);
                //}

                // compute cartesian position of tool
                T_C = T;// * T_T;

                // calculate transpose of jacobian
                jT = jac.data.transpose();

                // calculate inverse of manipulator mass matrix
                Mi = mass_.inverse();
                // calculate jacobian pseudo inverse
                Ji = Mi * jT * (jac.data * Mi * jT).inverse();
                // calculate null-space projection matrix
                N = (Eigen::Matrix<double, 7, 7>::Identity() - Ji * jac.data);
                // calculate cartesian mass matrix
                A = (jac.data * Mi * jT).inverse();

                // compute damping matrix
                es.compute(cart_stiffness_.asDiagonal(), A);
                K0 = es.eigenvalues();
                Q = es.eigenvectors().inverse();

                Dc = Q.transpose() * cart_damping_.asDiagonal() * K0.cwiseSqrt().asDiagonal() * Q;

                // compute length of spring
                T_S = T_C.Inverse() * T_D_;

                T_S.M.GetQuaternion(e(0), e(1), e(2), e(3));
                
                ROS_DEBUG_THROTTLE_NAMED(0.1,"cart","lwr_ctrl %s:kuka T_S %f, %f , %f, \n%f, %f, %f,\n %f, %f, %f,\n p %f, %f, %f", model_name_.c_str(), T_S.M.data[0],
                                        T_S.M.data[1], T_S.M.data[2],
                                        T_S.M.data[3], T_S.M.data[4],
                                        T_S.M.data[5], T_S.M.data[6],
                                        T_S.M.data[7], T_S.M.data[8],
                                        T_S.p.x(), T_S.p.y() , T_S.p.z());

                // calculate spring force
                F(0) = cart_stiffness_(0) * T_S.p[0];
                F(1) = cart_stiffness_(1) * T_S.p[1];
                F(2) = cart_stiffness_(2) * T_S.p[2];

                F(3) = cart_stiffness_(3) * e(0);
                F(4) = cart_stiffness_(4) * e(1);
                F(5) = cart_stiffness_(5) * e(2);

                // compute damping force
                F(0) -= Dc.diagonal()(0) * cart_twist.vel(0);
                F(1) -= Dc.diagonal()(1) * cart_twist.vel(1);
                F(2) -= Dc.diagonal()(2) * cart_twist.vel(2);

                F(3) -= Dc.diagonal()(3) * cart_twist.rot(0);
                F(4) -= Dc.diagonal()(4) * cart_twist.rot(1);
                F(5) -= Dc.diagonal()(5) * cart_twist.rot(2);


                // -------------------------------------------
                // add external wrench command
                F += ext_tcp_ft_;

                ROS_DEBUG_THROTTLE_NAMED(0.1, "cart","lwr_control %s:kuka F  %f, %f, %f, %f, %f, %f", model_name_.c_str(), F(0),
                  F(1),F(2),F(3),F(4),F(5));

                // transform cartesian force to joint torques
                tau = jT * F;
                // project stiffness to joint space for local stiffness control
                Kj = jT * cart_stiffness_.asDiagonal() * jac.data;

                ROS_DEBUG_THROTTLE_NAMED(0.1, "cart","lwr_control %s:kuka cart trq cmd %f, stiffness %f", model_name_.c_str(), tau(0), Kj(0,0));
                for(unsigned int i = 0; i < LBR_MNJ; i++) {
                  trq_cmd_(i) = tau(i);
                  stiffness_(i) = Kj(i, i);
                  damping_(i) = 0.0;
                }


                // compute the torque
                trq_ = stiffness_.asDiagonal() * (joint_pos_cmd_ - joint_pos_) - damping_.asDiagonal() * joint_vel_ + trq_cmd_;


                for(unsigned int i = 0; i< LBR_MNJ; i++) {
                  joints_[i]->SetForce(0, trq_(i) + grav(i));
                }
                ROS_DEBUG_STREAM_THROTTLE_NAMED(5.0, "krl", "lwr_ctrl " << model_name_ << "cartesian control");
              }
            }
            else
            {
              ROS_DEBUG_STREAM_THROTTLE_NAMED(5.0, "krl", "lwr_ctrl " << model_name_ << "other control");
              // just set gravity compensation
              for(unsigned int i = 0; i< LBR_MNJ; i++)
                joints_[i]->SetForce(0, grav(i));
            }
          }
        }
        else
        {
          Freeze(joint_pos_, grav);
        }

        // handle datagram validity check
        if(cnt < 20) {
          ++cnt;
        }
      }
    }
  }
  else {
    if(cnt > 0)
      --cnt;
    if(cnt <= 10)
      Freeze(joint_pos_, grav);
  }

  ROS_DEBUG_THROTTLE_NAMED(0.1, "joint", "lwr_ctrl %s :kuka pos cmd %f pos current %f vel curr %f trq_cmd %f trq %f, grav %f", model_name_.c_str(), joint_pos_cmd_(0), joint_pos_(0), joint_vel_(0), trq_cmd_(0), trq_(0), grav(0));
}

bool LWRController::DriveOnCb(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res)
{
  drive_on_ = true;
  ROS_DEBUG("%s drive on request", model_name_.c_str());
  return true;
}

bool LWRController::DriveOffCb(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res)
{
  ROS_DEBUG("%s drive off request", model_name_.c_str());
  // save the current position as brake position;
  for(unsigned int i = 0; i< 7; i++)
  {
    freeze_pos_(i) = joint_pos_(i);
  }
  drive_on_ = false;
  brakes_on_ = true;
  freeze_ = true;
  //m_msr_data.robot.power = 0x0;
  //m_msr_data.intf.state = FRI_STATE_MON;
  return true;
}

void LWRController::Freeze(Eigen::Matrix<double, 7, 1> &pos, KDL::JntArray &grav)
{
  if(!freeze_)
  {
    ROS_DEBUG("lwr freeze");
    for(unsigned int i = 0; i< LBR_MNJ; i++) {
      freeze_pos_(i) = pos(i);
    }
    freeze_ = true;
  }

  ROS_DEBUG_STREAM_THROTTLE(5.0,"lwr_ctrl " << model_name_ << " lwr enforcing brake_pos");
  //set the robot at same angles as brake pos
  for(unsigned int i = 0; i< LBR_MNJ; i++) {
    joints_[i]->SetPosition(0, freeze_pos_(i));
    joints_[i]->SetForce(0, grav(i));  // needed for recovery otherwise forces builds up
    joints_[i]->SetVelocity(0, 0.0);
  }
}

bool LWRController::isValidRotation(KDL::Rotation &rot)
{
    double det =
      rot.data[0] * rot.data[4] * rot.data[8]
     +rot.data[1] * rot.data[5] * rot.data[6]
     +rot.data[2] * rot.data[3] * rot.data[7]
     -rot.data[2] * rot.data[4] * rot.data[6]
     -rot.data[1] * rot.data[3] * rot.data[8]
     -rot.data[0] * rot.data[5] * rot.data[7];

    return (std::fabs(det)-1 < EPSILON_DET);
}

GZ_REGISTER_MODEL_PLUGIN(LWRController);

}

