void Joystick::jsCallback(const ardrone_autonomy::Navdata imu)
{
}
Joystick::Joystick(ros::NodeHandle &n) : nh_(n){
    controller_subscriber = n.subscribe("/ardrone/navdata", 10, &Joystick::jsCallback, this);
}

