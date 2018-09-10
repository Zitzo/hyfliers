//
//
//
//
//
//

#include "PidTuneGui.h"
#include <pid_tune_gui/Float32Param.h>
#include <QTime>
#include <std_srvs/Trigger.h>

PidTuneGui::PidTuneGui(std::string _refTopic, std::string _valTopic, std::vector<std::pair<std::string, std::string>> _pidBaseTopics, QWidget *parent): QMainWindow(parent) {
    mCentralWidget = new QWidget();
    mMainLayout = new QHBoxLayout();
    mCentralWidget->setLayout(mMainLayout);
    setCentralWidget(mCentralWidget);
    this->setWindowTitle("PID Tune Gui");

    // Add text entries for topics     
    mButtonsLayout = new QVBoxLayout();

    QHBoxLayout *refLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(refLayout);
    mRefEdit = new QLineEdit(_valTopic.c_str());
    mRefEdit->setMaximumWidth(300);
    auto refText = new QLineEdit("Ref. position");
    refText->setMaximumWidth(100);
    refText->setEnabled(false);
    refLayout->addWidget(refText);
    refLayout->addWidget(mRefEdit);

    QHBoxLayout *valLayout = new QHBoxLayout();
    mButtonsLayout->addLayout(valLayout);
    mPosEdit = new QLineEdit(_refTopic.c_str());
    mPosEdit->setMaximumWidth(300);
    auto posText = new QLineEdit("Current position");
    posText->setMaximumWidth(100);
    posText->setEnabled(false);
    valLayout->addWidget(posText);
    valLayout->addWidget(mPosEdit);

    mButtonsOptionsLayout = new QHBoxLayout();
    mChangeSubscribers = new QPushButton("Change subscribers");
    mButtonsOptionsLayout->addWidget(mChangeSubscribers);
    connect(mChangeSubscribers, SIGNAL (released()), this, SLOT (connectTopics()));

    mAnalisysWindowButton = new QPushButton("Analisys Window");
    mButtonsOptionsLayout->addWidget(mAnalisysWindowButton);
    connect(mAnalisysWindowButton, SIGNAL (released()), this, SLOT (openAnalysisWindow()));
    mButtonsLayout->addLayout(mButtonsOptionsLayout);

    // BUTTONS FOR K: X, Y, Z
    mButtonsLayoutFilaK = new QHBoxLayout();
    for(auto &baseTopic: _pidBaseTopics){
        mButtonsModifyXK = new QVBoxLayout();
        mButtonsModifyXK->addLayout(new ParamWidget(baseTopic.first+" kp",     baseTopic.second+"/kp",          0.8f , baseTopic.second+"/kp"));
        mButtonsModifyXK->addLayout(new ParamWidget(baseTopic.first+" ki",     baseTopic.second+"/ki",          0.01f, baseTopic.second+"/ki"));
        mButtonsModifyXK->addLayout(new ParamWidget(baseTopic.first+" kd",     baseTopic.second+"/kd",          0.7f , baseTopic.second+"/kd"));
        mButtonsModifyXK->addLayout(new ParamWidget(baseTopic.first+" sat",    baseTopic.second+"/saturation",  0.0f,  baseTopic.second+"/saturation"));
        mButtonsModifyXK->addLayout(new ParamWidget(baseTopic.first+" wind",   baseTopic.second+"/windup",      0.0f,  baseTopic.second+"/windup"));
        mButtonsLayoutFilaK->addLayout(mButtonsModifyXK);
    }
    mButtonsLayout->addLayout(mButtonsLayoutFilaK);
    // BUTTONS FOR MOVE
    mMainLayout->addLayout(mButtonsLayout); 
    
    // BUTTON AND ERRORS FOR ANALYSIS
    mLayoutFilaErr = new QHBoxLayout();

    mButtonsTime = new QVBoxLayout();
    mTimeMarkX = new QPushButton("Time Mark X");
    mTimeMarkX->setMaximumWidth(100);
    mButtonsTime->addWidget(mTimeMarkX);
    connect(mTimeMarkX, SIGNAL (released()), this, SLOT (activateMarkX()));

    mTimeMarkY = new QPushButton("Time Mark Y");
    mTimeMarkY->setMaximumWidth(100);
    mButtonsTime->addWidget(mTimeMarkY);
    connect(mTimeMarkY, SIGNAL (released()), this, SLOT (activateMarkY()));

    mTimeMarkZ = new QPushButton("Time Mark Z");
    mTimeMarkZ->setMaximumWidth(100);
    mButtonsTime->addWidget(mTimeMarkZ);
    connect(mTimeMarkZ, SIGNAL (released()), this, SLOT (activateMarkZ()));

    mLayoutFilaErr->addLayout(mButtonsTime);
    
    // Error
    mAnalysisTimeError = new QVBoxLayout();

    mNameError= new QLineEdit("Error");
    mNameError->setEnabled(false);
    mNameError->setMaximumWidth(50);
    mAnalysisTimeError->addWidget(mNameError);

    mTextErrorX = new QLineEdit();
    mTextErrorX->setText(QString::number(mErrorX,'g',7));
    mTextErrorX->setEnabled(false);
    mTextErrorX->setMaximumWidth(90);
    mAnalysisTimeError->addWidget(mTextErrorX);

    mTextErrorY = new QLineEdit();
    mTextErrorY->setText(QString::number(mErrorY,'g',7));
    mTextErrorY->setEnabled(false);
    mTextErrorY->setMaximumWidth(90);
    mAnalysisTimeError->addWidget(mTextErrorY);

    mTextErrorZ = new QLineEdit();
    mTextErrorZ->setText(QString::number(mErrorZ,'g',7));
    mTextErrorZ->setEnabled(false);
    mTextErrorZ->setMaximumWidth(90);
    mAnalysisTimeError->addWidget(mTextErrorZ);

    mLayoutFilaErr->addLayout(mAnalysisTimeError);

    // Percentage Error
    mAnalysisTimePercentage = new QVBoxLayout();

    mNamePercentage= new QLineEdit(" % ");
    mNamePercentage->setEnabled(false);
    mNamePercentage->setMaximumWidth(50);
    mAnalysisTimePercentage->addWidget(mNamePercentage);

    mTextPercentageX = new QLineEdit();
    mTextPercentageX->setText(QString::number(mPercentageX,'g',7));
    mTextPercentageX->setEnabled(false);
    mTextPercentageX->setMaximumWidth(90);
    mAnalysisTimePercentage->addWidget(mTextPercentageX);

    mTextPercentageY = new QLineEdit();
    mTextPercentageY->setText(QString::number(mPercentageY,'g',7));
    mTextPercentageY->setEnabled(false);
    mTextPercentageY->setMaximumWidth(90);
    mAnalysisTimePercentage->addWidget(mTextPercentageY);

    mTextPercentageZ = new QLineEdit();
    mTextPercentageZ->setText(QString::number(mPercentageZ,'g',7));
    mTextPercentageZ->setEnabled(false);
    mTextPercentageZ->setMaximumWidth(90);
    mAnalysisTimePercentage->addWidget(mTextPercentageZ);

    mLayoutFilaErr->addLayout(mAnalysisTimePercentage);

    mButtonsLayout->addLayout( mLayoutFilaErr);
    mMainLayout->addLayout(mButtonsLayout);

    // Time Marks
    mLayoutFilaTimeMarks = new QHBoxLayout();
    
    mAnalysisTimeMarkT0 = new QVBoxLayout();
    mNameMarkXT0 = new QLineEdit("X T0");
    mNameMarkXT0->setEnabled(false);
    mNameMarkXT0->setMaximumWidth(50);
    mAnalysisTimeMarkT0->addWidget(mNameMarkXT0);

    mTextMarkXT0 = new QLineEdit();
    mTextMarkXT0->setText(QString::number(mMarkXT0,'g',7));
    mTextMarkXT0->setEnabled(false);
    mTextMarkXT0->setMaximumWidth(70);
    mAnalysisTimeMarkT0->addWidget(mTextMarkXT0);

    mNameMarkYT0 = new QLineEdit("Y T0");
    mNameMarkYT0->setEnabled(false);
    mNameMarkYT0->setMaximumWidth(50);
    mAnalysisTimeMarkT0->addWidget(mNameMarkYT0);

    mTextMarkYT0 = new QLineEdit();
    mTextMarkYT0->setText(QString::number(mMarkYT0,'g',7));
    mTextMarkYT0->setEnabled(false);
    mTextMarkYT0->setMaximumWidth(70);
    mAnalysisTimeMarkT0->addWidget(mTextMarkYT0);

    mNameMarkZT0 = new QLineEdit("Z T0");
    mNameMarkZT0->setEnabled(false);
    mNameMarkZT0->setMaximumWidth(50);
    mAnalysisTimeMarkT0->addWidget(mNameMarkZT0);

    mTextMarkZT0 = new QLineEdit();
    mTextMarkZT0->setText(QString::number(mMarkZT0,'g',7));
    mTextMarkZT0->setEnabled(false);
    mTextMarkZT0->setMaximumWidth(70);
    mAnalysisTimeMarkT0->addWidget(mTextMarkZT0);

    mLayoutFilaTimeMarks->addLayout( mAnalysisTimeMarkT0 );

    mAnalysisTimeMarkT1 = new QVBoxLayout();

    mAnalysisTimeMarkT1 = new QVBoxLayout();
    mNameMarkXT1 = new QLineEdit("X T1");
    mNameMarkXT1->setEnabled(false);
    mNameMarkXT1->setMaximumWidth(50);
    mAnalysisTimeMarkT1->addWidget(mNameMarkXT1);

    mTextMarkXT1 = new QLineEdit();
    mTextMarkXT1->setText(QString::number(mMarkXT1,'g',7));
    mTextMarkXT1->setEnabled(false);
    mTextMarkXT1->setMaximumWidth(70);
    mAnalysisTimeMarkT1->addWidget(mTextMarkXT1);

    mNameMarkYT1 = new QLineEdit("Y T1");
    mNameMarkYT1->setEnabled(false);
    mNameMarkYT1->setMaximumWidth(50);
    mAnalysisTimeMarkT1->addWidget(mNameMarkYT1);

    mTextMarkYT1 = new QLineEdit();
    mTextMarkYT1->setText(QString::number(mMarkYT1,'g',7));
    mTextMarkYT1->setEnabled(false);
    mTextMarkYT1->setMaximumWidth(70);
    mAnalysisTimeMarkT1->addWidget(mTextMarkYT1);

    mNameMarkZT1 = new QLineEdit("Z T1");
    mNameMarkZT1->setEnabled(false);
    mNameMarkZT1->setMaximumWidth(50);
    mAnalysisTimeMarkT1->addWidget(mNameMarkZT1);

    mTextMarkZT1 = new QLineEdit();
    mTextMarkZT1->setText(QString::number(mMarkZT1,'g',7));
    mTextMarkZT1->setEnabled(false);
    mTextMarkZT1->setMaximumWidth(70);
    mAnalysisTimeMarkT1->addWidget(mTextMarkZT1);

    mLayoutFilaTimeMarks->addLayout( mAnalysisTimeMarkT1 );

    mAnalysisTimeMarkT2 = new QVBoxLayout();

    mAnalysisTimeMarkT2 = new QVBoxLayout();
    mNameMarkXT2 = new QLineEdit("X T2");
    mNameMarkXT2->setEnabled(false);
    mNameMarkXT2->setMaximumWidth(50);
    mAnalysisTimeMarkT2->addWidget(mNameMarkXT2);

    mTextMarkXT2 = new QLineEdit();
    mTextMarkXT2->setText(QString::number(mMarkXT2,'g',7));
    mTextMarkXT2->setEnabled(false);
    mTextMarkXT2->setMaximumWidth(70);
    mAnalysisTimeMarkT2->addWidget(mTextMarkXT2);

    mNameMarkYT2 = new QLineEdit("Y T2");
    mNameMarkYT2->setEnabled(false);
    mNameMarkYT2->setMaximumWidth(50);
    mAnalysisTimeMarkT2->addWidget(mNameMarkYT2);

    mTextMarkYT2 = new QLineEdit();
    mTextMarkYT2->setText(QString::number(mMarkYT2,'g',7));
    mTextMarkYT2->setEnabled(false);
    mTextMarkYT2->setMaximumWidth(70);
    mAnalysisTimeMarkT2->addWidget(mTextMarkYT2);

    mNameMarkZT2 = new QLineEdit("Z T2");
    mNameMarkZT2->setEnabled(false);
    mNameMarkZT2->setMaximumWidth(50);
    mAnalysisTimeMarkT2->addWidget(mNameMarkZT2);

    mTextMarkZT2 = new QLineEdit();
    mTextMarkZT2->setText(QString::number(mMarkZT2,'g',7));
    mTextMarkZT2->setEnabled(false);
    mTextMarkZT2->setMaximumWidth(70);
    mAnalysisTimeMarkT2->addWidget(mTextMarkZT2);

    mLayoutFilaTimeMarks->addLayout( mAnalysisTimeMarkT2 );

    mButtonsLayout->addLayout( mLayoutFilaTimeMarks);
    mMainLayout->addLayout(mButtonsLayout);

    /// GRAPH X
    mGraphLayout = new QVBoxLayout();
    mGraphPositionX = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPositionX);
    mMainLayout->addLayout(mGraphLayout);

    mGraphPositionX->addGraph(); // red line X
    QPen pen;
    pen.setWidthF(4);
    pen.setColor(QColor(255, 0, 0));
    mGraphPositionX->graph(0)->setPen(pen);

    mGraphPositionX->addGraph(); // red line X ref
    pen.setColor(QColor(255, 0, 0));
    pen.setStyle(Qt::DotLine);
    mGraphPositionX->graph(1)->setPen(pen);
    
    QSharedPointer<QCPAxisTickerTime> mTimeTicker(new QCPAxisTickerTime);
    mTimeTicker->setTimeFormat("%h:%m:%s");
    mGraphPositionX->xAxis->setTicker(mTimeTicker);
    mGraphPositionX->yAxis->setRange(-1.0, 1.0);

    // GRAPH Y
    mGraphPositionY = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPositionY);
    mGraphPositionY->addGraph(); // green line Y
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::SolidLine);
    mGraphPositionY->graph(0)->setPen(pen);

    mGraphPositionY->addGraph(); // green line Y ref
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::DotLine);
    mGraphPositionY->graph(1)->setPen(pen);

    mGraphPositionY->xAxis->setTicker(mTimeTicker);
    mGraphPositionY->yAxis->setRange(-1.0, 1.0);

    // GRAPH Z
    mGraphPositionZ = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPositionZ);
    mGraphPositionZ->addGraph(); // blue line Z
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::SolidLine);
    mGraphPositionZ->graph(0)->setPen(pen);

    mGraphPositionZ->addGraph(); // blue line Z ref
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::DotLine);
    mGraphPositionZ->graph(1)->setPen(pen);

    mGraphPositionZ->xAxis->setTicker(mTimeTicker);
    mGraphPositionZ->yAxis->setRange(-1.0, 1.0);
    
    connect(mGraphPositionX->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionX->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPositionX->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionX->yAxis2, SLOT(setRange(QCPRange)));
     
    connect(mGraphPositionY->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionY->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPositionY->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionY->yAxis2, SLOT(setRange(QCPRange)));

    connect(mGraphPositionZ->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionZ->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPositionZ->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionZ->yAxis2, SLOT(setRange(QCPRange)));

    mDataTimer = new QTimer(this);
    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(realTimePlot()));
    mDataTimer->start(0);

    mDataTimerMark = new QTimer(this);
    connect(mDataTimerMark, SIGNAL(timeout()), this, SLOT(realTimeMark()));
    mDataTimerMark->start(0);

}


void PidTuneGui::closeEvent(QCloseEvent *event) {

}

void PidTuneGui::activateMarkX(){
        
    // Change Value of Marks sequentially
    if(mFlagMarkX){
        mMarkXT1 = mMarkXT1Aux;
        mTextMarkXT1->setText(QString::number(mMarkXT1,'g',7));
        mFlagMarkX = false;
    }
    else{
        mMarkXT2 = mMarkXT2Aux;
        mTextMarkXT2->setText(QString::number(mMarkXT2,'g',7));
        mFlagMarkX = true;
        mFinishAnalysisX = true;
    }
    
}

void PidTuneGui::activateMarkY(){
    
// Change Value of Marks sequentially
if(mFlagMarkY){
    mMarkYT1 = mMarkYT1Aux;
    mTextMarkYT1->setText(QString::number(mMarkYT1,'g',7));
    mFlagMarkY = false;
}
else{
    mMarkYT2 = mMarkYT2Aux;
    mTextMarkYT2->setText(QString::number(mMarkYT2,'g',7));
    mFlagMarkY = true;
    mFinishAnalysisY = true;
}

}

void PidTuneGui::activateMarkZ(){
    
// Change Value of Marks sequentially
if(mFlagMarkZ){
    mMarkZT1 = mMarkZT1Aux;
    mTextMarkZT1->setText(QString::number(mMarkZT1,'g',7));
    mFlagMarkZ = false;
}
else{
    mMarkZT2 = mMarkZT2Aux;
    mTextMarkZT2->setText(QString::number(mMarkZT2,'g',7));
    mFlagMarkZ = true;
    mFinishAnalysisZ = true;
}

}

void PidTuneGui::realTimeMark(){
    
    static QTime timeMark(QTime::currentTime());
    // calculate two new data points:
    float keyMark = timeMark.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static float lastPointKeyMark = 0;
    static float changeReferenceX = mLastRefX;
    static float changeReferenceY = mLastRefY;
    static float changeReferenceZ = mLastRefZ;

    if (keyMark-lastPointKeyMark > 0.005) {    

        // Almacenamos el tiempo cada que llevamos cada 2 ms en las variables y se actualiza secuencialmente cuando pulsamos el botón
        lastPointKeyMark = keyMark;
        mMarkXT1Aux = lastPointKeyMark;
        mMarkXT2Aux = lastPointKeyMark;

        mMarkYT1Aux = lastPointKeyMark;
        mMarkYT2Aux = lastPointKeyMark;

        mMarkZT1Aux = lastPointKeyMark;
        mMarkZT2Aux = lastPointKeyMark;

        mErrorX = mLastRefX - mLastX;
        mErrorY = mLastRefY - mLastY;
        mErrorZ = mLastRefZ - mLastZ;
        mTextErrorX->setText(QString::number(mErrorX,'g',7));
        mTextErrorY->setText(QString::number(mErrorY,'g',7));
        mTextErrorZ->setText(QString::number(mErrorZ,'g',7));
    
        mPercentageX = std::abs(mLastX)/mStepSizeX;
        mPercentageY = std::abs(mLastY)/mStepSizeY;
        mPercentageZ = std::abs(mLastZ)/mStepSizeZ;
        mTextPercentageX->setText(QString::number(mPercentageX,'g',7));
        mTextPercentageY->setText(QString::number(mPercentageY,'g',7));
        mTextPercentageZ->setText(QString::number(mPercentageZ,'g',7));
        
        if(changeReferenceX != mLastRefX){
            std::cout << " Saving X..." << std::endl;
            mTextMarkXT1->setText(QString::number(0,'g',7));
            mTextMarkXT2->setText(QString::number(0,'g',7));
            mRefToSave = 1;
            mAnalysisRef.clear();
            mAnalysisData.clear();
            mAnalysisKey.clear();
            mAnalysisKeyTs.clear();
            mStepSizeX = std::abs(mLastRefX - changeReferenceX);
            mMarkXT0 = lastPointKeyMark;
            mTextMarkXT0->setText(QString::number(mMarkXT0,'g',7));
            changeReferenceX = mLastRefX;
        }
        if(changeReferenceY != mLastRefY){
            std::cout << " Saving Y..." << std::endl;
            mTextMarkYT1->setText(QString::number(0,'g',7));
            mTextMarkYT2->setText(QString::number(0,'g',7));
            mRefToSave = 2;
            mAnalysisRef.clear();
            mAnalysisData.clear();
            mAnalysisKey.clear();
            mAnalysisKeyTs.clear();
            mStepSizeY = std::abs(mLastRefY - changeReferenceY);
            mMarkYT0 = lastPointKeyMark;
            mTextMarkYT0->setText(QString::number(mMarkYT0,'g',7));
            changeReferenceY = mLastRefY;
        }
        if(changeReferenceZ != mLastRefZ){
            std::cout << " Saving Z..." << std::endl;
            mTextMarkZT1->setText(QString::number(0,'g',7));
            mTextMarkZT2->setText(QString::number(0,'g',7));
            mRefToSave = 3;
            mAnalysisRef.clear();
            mAnalysisData.clear();
            mAnalysisKey.clear();
            mAnalysisKeyTs.clear();
            mStepSizeZ = std::abs(mLastRefZ - changeReferenceZ);
            mMarkZT0 = lastPointKeyMark;
            mTextMarkZT0->setText(QString::number(mMarkZT0,'g',7));
            changeReferenceZ = mLastRefZ;
        }

        // CASES FOR STORE DATA IF CHANGE REF IN X, Y OR Z
        if(mRefToSave == 1){
            mAnalysisRef.push_back(mLastRefX);
            mAnalysisData.push_back(mLastX);
            mAnalysisKey.push_back(lastPointKeyMark);
        }
        else if(mRefToSave == 2){
            mAnalysisRef.push_back(mLastRefY);
            mAnalysisData.push_back(mLastY);
            mAnalysisKey.push_back(lastPointKeyMark);
        }
        else if(mRefToSave == 3){
            mAnalysisRef.push_back(mLastRefY);
            mAnalysisData.push_back(mLastY);
            mAnalysisKey.push_back(lastPointKeyMark);
        }
        else{
            // do nothing if dont change ref in X, Y or Z 
            // do nothing if you have T2 and want to finish get data
        }

        if(mFinishAnalysisX){
            mRefToSave = 0;
            mFinishAnalysisX = false;
            mAnalysisKeyTs.push_back(mMarkXT0);
            mAnalysisKeyTs.push_back(mMarkXT1);
            mAnalysisKeyTs.push_back(mMarkXT2);
            std::cout << " Finish Analysis!" << std::endl;
        }
        else if(mFinishAnalysisY){
            mRefToSave = 0;
            mFinishAnalysisY = false;
            mAnalysisKeyTs.push_back(mMarkYT0);
            mAnalysisKeyTs.push_back(mMarkYT1);
            mAnalysisKeyTs.push_back(mMarkYT2);
            std::cout << " Finish Analysis!" << std::endl;
        }
        else if(mFinishAnalysisZ){
            mRefToSave = 0;
            mFinishAnalysisZ = false;
            mAnalysisKeyTs.push_back(mMarkZT0);
            mAnalysisKeyTs.push_back(mMarkZT1);
            mAnalysisKeyTs.push_back(mMarkZT2);
            std::cout << " Finish Analysis!" << std::endl;
        }
        else{
            // do nothing
        }



    }

}


void PidTuneGui::realTimePlot(){
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.005) { // at most add point every 2 ms
      // add data to lines:
      mGraphPositionX->graph(0)->addData(key, mLastX);
      mGraphPositionX->graph(1)->addData(key, mLastRefX);
      
      mGraphPositionY->graph(0)->addData(key, mLastY);
      mGraphPositionY->graph(1)->addData(key, mLastRefY);

      mGraphPositionZ->graph(0)->addData(key, mLastZ);
      mGraphPositionZ->graph(1)->addData(key, mLastRefZ);

      lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    
    double min = mLastX-1 < mLastRefX - 1? mLastX -1: mLastRefX -1;
    double max = mLastX+1 > mLastRefX + 1? mLastX +1: mLastRefX +1;
    mGraphPositionX->yAxis->setRange(min, max);

    min = mLastY-1 < mLastRefY - 1? mLastY -1: mLastRefY -1;
    max = mLastY+1 > mLastRefY + 1? mLastY +1: mLastRefY +1;
    mGraphPositionY->yAxis->setRange(min, max);

    min = mLastZ-1 < mLastRefZ - 1? mLastZ -1: mLastRefZ -1;
    max = mLastZ+1 > mLastRefZ + 1? mLastZ +1: mLastRefZ +1;
    mGraphPositionZ->yAxis->setRange(min, max);

    mGraphPositionX->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPositionX->replot();

    mGraphPositionY->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPositionY->replot();

    mGraphPositionZ->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPositionZ->replot();
}

void PidTuneGui::connectTopics(){
    // other
    ros::NodeHandle n;
    mSubPos = n.subscribe<geometry_msgs::PoseStamped>(mPosEdit->text().toStdString(),1, &PidTuneGui::positionCallback, this);
    mSubRef = n.subscribe<geometry_msgs::PoseStamped>(mRefEdit->text().toStdString(), 1,&PidTuneGui::referenceCallback, this);
    //mSubXKp = n.subscribe<std_msgs::Float32>("/PID_x/kp", 1,&PidTuneGui::changePIDCallback, this);

}

//void PidTuneGui::changePIDCallback(const std_msgs::Float32::ConstPtr &_data){}



void PidTuneGui::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &_data){
    mLastX = _data->pose.position.x;
    mLastY = _data->pose.position.y;
    mLastZ = _data->pose.position.z;
}

void PidTuneGui::referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &_data){
    mLastRefX = _data->pose.position.x;
    mLastRefY = _data->pose.position.y;
    mLastRefZ = _data->pose.position.z;
}

void PidTuneGui::openAnalysisWindow(){
    if(mLastAnalisysWindow != nullptr){
        mLastAnalisysWindow->close();
        mLastAnalisysWindow = nullptr;
    }
    std::cout << mAnalysisRef.size() << " | " << mAnalysisData.size() << " | " << mAnalysisKey.size() << " | " << mAnalysisKeyTs.size() << std::endl;
    mLastAnalisysWindow = new AnalisysWindow(mAnalysisRef,mAnalysisData,mAnalysisKey,mAnalysisKeyTs);
    mLastAnalisysWindow->setModal(true);
    mLastAnalisysWindow->exec();
}
