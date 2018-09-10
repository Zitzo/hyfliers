//
//
//
//
//

#ifndef PIDTUNEGUI_H_
#define PIDTUNEGUI_H_

    #include <ros/ros.h>
    #include <QHBoxLayout>
    #include <QVBoxLayout>
    #include <QMainWindow>
    #include <QLineEdit>
    #include <QTextEdit>
    #include "ParamWidget.h"
    #include <map>
    #include <geometry_msgs/PoseStamped.h>
    #include "qcustomplot.h"
    #include "AnalisysWindow.h"
    #include <vector>

    class PidTuneGui : public QMainWindow {
        Q_OBJECT
    public:
        PidTuneGui(std::string _refTopic, std::string _valTopic,  std::vector<std::pair<std::string, std::string>> _pidBaseTopics, QWidget *parent = 0);
        virtual ~PidTuneGui(){};

    protected:
        void closeEvent(QCloseEvent *event) override;
        
    private slots:
        void realTimePlot();
        void realTimeMark();
        void connectTopics();
        void openAnalysisWindow();
        void activateMarkX();
        void activateMarkY();
        void activateMarkZ();

    private:
        void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &_data);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &_data);
        //void changePIDCallback(const std_msgs::Float32::ConstPtr &_data);
        
    private:
        QHBoxLayout *mMainLayout, *mButtonsOptionsLayout;

        QVBoxLayout *mButtonsModifySat, *mButtonsModifyWP, *mButtonsModifyXK, *mButtonsModifyYK, *mButtonsModifyZK, *mButtonsModifyRef, *mButtonsTime, *mAnalysisTimeError, *mAnalysisTimePercentage, *mAnalysisTimeMarkT0, *mAnalysisTimeMarkT1, *mAnalysisTimeMarkT2;
        QHBoxLayout *mButtonsLayoutFilaK, *mButtonsLayoutFilaSW, *mButtonsLayoutFilaRef, *mLayoutFilaErr, *mLayoutFilaTimeMarks;

        QVBoxLayout *mButtonsLayout, *mGraphLayout;
        QWidget     *mCentralWidget;
        QLineEdit *mPosEdit, *mRefEdit;
        QLineEdit *mNameError, *mNamePercentage, *mTextErrorX, *mTextErrorY, *mTextErrorZ, *mTextPercentageX, *mTextPercentageY, *mTextPercentageZ, *mNameMarkXT0, *mNameMarkYT0, *mNameMarkZT0, *mNameMarkXT1, *mNameMarkYT1, *mNameMarkZT1, *mNameMarkXT2, *mNameMarkYT2, *mNameMarkZT2, *mTextMarkXT0, *mTextMarkYT0, *mTextMarkZT0, *mTextMarkXT1, *mTextMarkYT1, *mTextMarkZT1, *mTextMarkXT2, *mTextMarkYT2, *mTextMarkZT2;
        QPushButton *mChangeSubscribers, *mTimeMarkX, *mTimeMarkY, *mTimeMarkZ;
       

        QCustomPlot *mGraphPositionX, *mGraphPositionY, *mGraphPositionZ;
        QTimer *mDataTimer, *mDataTimerMark;
        ros::Subscriber mSubPos, mSubRef, mSubXKp;
        float mLastX = 0, mLastY = 0, mLastZ = 0, mLastRefX = 0, mLastRefY = 0, mLastRefZ = 0;

        QPushButton *mAnalisysWindowButton;
        AnalisysWindow *mLastAnalisysWindow = nullptr;

        float mErrorX = 0, mErrorY = 0, mErrorZ = 0, mPercentageX = 0, mPercentageY = 0, mPercentageZ = 0;
        float mMarkXT0 = 0, mMarkXT1 = 0, mMarkXT2 = 0, mMarkYT0 = 0, mMarkYT1 = 0, mMarkYT2 = 0, mMarkZT0 = 0, mMarkZT1 = 0, mMarkZT2 = 0;
        float mMarkXT1Aux = 0, mMarkXT2Aux = 0, mMarkYT1Aux = 0, mMarkYT2Aux = 0, mMarkZT1Aux = 0, mMarkZT2Aux = 0;
        int mFlagMarkX = 0, mFlagMarkY = 0, mFlagMarkZ = 0;
        float mStepSizeX = 0.0, mStepSizeY = 0.0, mStepSizeZ = 0.0;

        std::vector<float> mAnalysisRef, mAnalysisData, mAnalysisKey, mAnalysisKeyTs; 
        int mRefToSave = 0; 
        bool mFinishAnalysisX = false, mFinishAnalysisY = false, mFinishAnalysisZ = false;
    };

#endif
