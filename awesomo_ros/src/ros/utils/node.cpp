#include "awesomo_ros/utils/node.hpp"


namespace awesomo {

ROSNode::ROSNode(int argc, char **argv) {
  this->configured = false;
  this->debug_mode = false;
  this->sim_mode = false;

  this->argc = argc;
  this->argv = argv;

  this->ros_node_name = "";
  this->ros_seq = 0;
  this->ros_rate = NULL;
}

ROSNode::~ROSNode(void) {
  ::ros::shutdown();
}

int ROSNode::configure(const std::string node_name, int hz) {
  // clang-format off
  if (::ros::isInitialized() == false) {
    ::ros::init(
      this->argc,
      this->argv,
      node_name,
      ::ros::init_options::NoSigintHandler
    );
  }
  // clang-format on

  // initialize
  this->ros_node_name = node_name;
  this->ros_nh = new ::ros::NodeHandle();
  this->ros_nh->getParam("/debug_mode", this->debug_mode);
  this->ros_nh->getParam("/sim_mode", this->sim_mode);
  this->ros_rate = new ::ros::Rate(hz);
  this->configured = true;

  return 0;
}

void ROSNode::shutdownCallback(const std_msgs::Bool &msg) {
  if (msg.data) {
    ::ros::shutdown();
  }
}

int ROSNode::registerShutdown(std::string topic) {
  bool retval;
  ::ros::Subscriber sub;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // register subscriber
  sub = this->ros_nh->subscribe(topic, 1, &ROSNode::shutdownCallback, this);
  this->ros_subs[topic] = sub;
}

int ROSNode::registerImagePublisher(const std::string &topic) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // image transport
  image_transport::ImageTransport it(*this->ros_nh);
  this->img_pub = it.advertise(topic, 1);

  return 0;
}

int ROSNode::registerLoopCallback(std::function<int(void)> cb) {
  this->loop_cb = cb;
  return 0;
}

int ROSNode::loop(void) {
  int retval;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // loop
  while (::ros::ok()) {
    // run loop callback
    if (this->loop_cb != nullptr) {
      retval = this->loop_cb();
      if (retval != 0) {
        return retval;
      }
    }

    // update
    ::ros::spinOnce();
    this->ros_seq++;
    this->ros_last_updated = ::ros::Time::now();
    this->ros_rate->sleep();
  }

  return 0;
}

}  // end of awesomo namespace
