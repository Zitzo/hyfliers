//
//
//
//
//


#include "ParamWidget.h"
#include <ros/ros.h>
#include <pid_tune_gui/Float32Param.h>
#include <std_msgs/Float32.h>

ParamWidget::ParamWidget(std::string _name, std::string _serviceTopic, float _default, std::string _paramSubTopic){
    mParamName = new QLineEdit(QString(_name.c_str()));
    mParamName->setEnabled(false);
    mParamName->setMaximumWidth(40);
    this->addWidget(mParamName);
    
    //mEntry = new QLineEdit(QString(std::to_string(_default).c_str()));
    mEntry = new QLineEdit();
    mEntry->setText(QString::number(_default,'g',3));
    mEntry->setMaximumWidth(50);
    //mEntry->setInputMask("0.000");
    this->addWidget(mEntry);
    
    mButton = new QPushButton((_name).c_str());
    mButton->setMaximumWidth(40);
    this->addWidget(mButton);

    mServiceTopic = _serviceTopic;
    connect(mButton, SIGNAL (released()), this, SLOT (sendCallback()));

    ros::NodeHandle nh;
    mParamSubscriber = nh.subscribe<std_msgs::Float32>(_paramSubTopic, 1, [&](const std_msgs::Float32::ConstPtr &_msg){
        mEntry->setText(QString::number(_msg->data,'g',3));
    });
}

void ParamWidget::sendCallback(){
    if(ros::isInitialized()){
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<pid_tune_gui::Float32Param>(mServiceTopic);
        pid_tune_gui::Float32Param srv;
        srv.request.param = mEntry->text().toFloat();
        client.call(srv);
    }
}
