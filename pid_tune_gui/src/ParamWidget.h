//
//
//
//
//

#ifndef PARAMWIDGET_H_
#define PARAMWIDGET_H_

    #include <QHBoxLayout>
    #include <QPushButton>
    #include <QLineEdit>
    #include <QGroupBox>
    #include <ros/ros.h>

    class ParamWidget : public QHBoxLayout {
        Q_OBJECT
    public:
        ParamWidget(std::string _name, std::string _serviceTopic, float _default = 0.0f, std::string _paramSubTopic = "");
        virtual ~ParamWidget(){};
    
        std::string topic() {return mServiceTopic;};
        float value(){return mEntry->text().toFloat();};
    private slots:
        void sendCallback();
    private:
        QLineEdit   *mEntry, *mParamName;
        QPushButton *mButton;
        std::string mServiceTopic;
        ros::Subscriber mParamSubscriber;
    };
    
#endif