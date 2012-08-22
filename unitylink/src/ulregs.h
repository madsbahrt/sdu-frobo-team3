#define IN 1
#define OUT 0

struct ULRegister {
  int id;
  int direction;
  ros::Publisher publisher;
  ros::Subscriber subscriber;
  std::string command;
  std::string data;
};

ULRegister ulregs[8];

