int main (int argc, char**argv)
{
}

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pipe_detection");
  ros::NodeHandle n("~");
  StateFilter sf(n);
  ros::spin();
  return 0;
}