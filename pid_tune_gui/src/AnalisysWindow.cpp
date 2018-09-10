//
//
//
//
//
//


#include "AnalisysWindow.h"

AnalisysWindow::AnalisysWindow(std::vector<float> _refs, 
    std::vector<float> _data, 
    std::vector<float> _ts, 
    std::vector<float> _keyTs, 
    QWidget *parent){
        mRefs = _refs;
        mData = _data;
        mTs = _ts;
        mKeyTs = _keyTs;

        mMainLayout = new QHBoxLayout();        
        this->setWindowTitle("Analisys window");        
        mPanel = new QVBoxLayout();
        mMainLayout->addLayout(mPanel);
        mGraph = new QCustomPlot();
        mMainLayout->addWidget(mGraph);

        buildPanel();      
        buildGraph();  
        this->setLayout(mMainLayout);

}

void AnalisysWindow::buildPanel(){

    mLayTs = new QHBoxLayout();

    mLEt0 = new QLineEdit(); mLEt0->setEnabled(false); mLEt0->setMaximumWidth(60); 
    mLayTs->addWidget(mLEt0);
    mLEt1 = new QLineEdit(); mLEt1->setEnabled(false); mLEt1->setMaximumWidth(60); 
    mLayTs->addWidget(mLEt1);
    mLEt2 = new QLineEdit(); mLEt2->setEnabled(false); mLEt2->setMaximumWidth(60); 
    mLayTs->addWidget(mLEt2);


    mLaytsig = new QHBoxLayout();
    mLEtsig = new QLineEdit(); mLEtsig->setEnabled(false); mLEtsig->setMaximumWidth(60); 
    mLaytsig->addWidget(mLEtsig);
    mButtsig = new QPushButton("t_sig"); mButtsig->setMaximumWidth(60); mLaytsig->addWidget(mButtsig);
    connect(mButtsig, SIGNAL (released()), this, SLOT (computeTsig()));
    
    mLayvmax1 = new QHBoxLayout();
    mLEvmax1 = new QLineEdit(); mLEvmax1->setEnabled(false); mLEvmax1->setMaximumWidth(60); 
    mLayvmax1->addWidget(mLEvmax1);
    mButvmax1 = new QPushButton("v_max1"); mButvmax1->setMaximumWidth(60); mLayvmax1->addWidget(mButvmax1);
    connect(mButvmax1, SIGNAL (released()), this, SLOT (computeMax1()));
    

    mLaytmax1  = new QHBoxLayout();
    mLEtmax1 = new QLineEdit(); mLEtmax1->setEnabled(false); mLEtmax1->setMaximumWidth(60); 
    mLaytmax1->addWidget(mLEtmax1);
    mButtmax1 = new QPushButton("t_max1"); mButtmax1->setEnabled(false); mButtmax1->setMaximumWidth(60); mLaytmax1->addWidget(mButtmax1);
    
    mLayavgPer = new QHBoxLayout();
    mLEavgPer = new QLineEdit(); mLEavgPer->setEnabled(false); mLEavgPer->setMaximumWidth(60); 
    mLayavgPer->addWidget(mLEavgPer);
    mButavgPer = new QPushButton("avg_perm"); mButavgPer->setMaximumWidth(60); mLayavgPer->addWidget(mButavgPer);
    connect(mButavgPer, SIGNAL (released()), this, SLOT (computeAvgPer()));

    mPanel->addLayout(mLayTs);
    mPanel->addLayout(mLaytsig);
    mPanel->addLayout(mLayvmax1);
    mPanel->addLayout(mLaytmax1);
    mPanel->addLayout(mLayavgPer);
}

void AnalisysWindow::buildGraph(){

    mGraph->addGraph(); // blue line
    QPen pen;
    pen.setWidthF(4);
    pen.setColor(QColor(0, 0, 255));
    mGraph->graph(0)->setPen(pen);

    mGraph->addGraph(); // red dot line ref
    pen.setColor(QColor(255, 0, 0));
    pen.setStyle(Qt::DotLine);
    mGraph->graph(1)->setPen(pen);
    connect(mGraph->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraph->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraph->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraph->yAxis2, SLOT(setRange(QCPRange)));

    for (long index=0; index<(long)mTs.size(); ++index) {
        mGraph->graph(0)->addData(mTs.at(index), mData.at(index));
        mGraph->graph(1)->addData(mTs.at(index), mRefs.at(index));

        double min = mData.at(0)-1 < mRefs.at(0) - 1? mData.at(0) -1: mRefs.at(0) -1;
        double max = mData.at(mTs.size()-1)+1 > mRefs.at(mTs.size()-1) + 1? mData.at(mTs.size()-1) +1: mRefs.at(mTs.size()-1) +1;
        mGraph->yAxis->setRange(min, max);

        mGraph->xAxis->setRange(mTs.at(0), mTs.at(mTs.size()-1));

        mGraph->replot(); 
    }

}

//---------------------------------------------------------------------------------------------------------------------
void AnalisysWindow::computeTsig(){
    double step =  mRefs[0]-mData[0];
    std::cout << step << std::endl;
    double t0 = mTs[0];
    for(unsigned i = 0; i < mData.size(); i++){
        if(mData[i] > 0.63*step){
            mLEtsig->setText(QString::number(mTs[i]-t0,'g',5));
            break;
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void AnalisysWindow::computeAvgPer(){
    int idT1;
    for(unsigned i = 0; i < mTs.size(); i++){
        if(mTs[i]> mKeyTs[1]){
            idT1 = i;
            break;
        }
    }
    double avg = 0;
    for(unsigned i = idT1; i < mData.size(); i++){
        avg += mData[i]-mRefs[0];
    }
    avg /= double(mTs.size() - idT1);
    mLEavgPer->setText(QString::number(avg,'g',5));
}

//---------------------------------------------------------------------------------------------------------------------
void AnalisysWindow::computeMax1(){

    int counterPositive = 0;
    double step = 10.0;
    for(unsigned i = 0; i < mData.size()-2*step; i = i+step){

        double curr  = 0;
        for(unsigned k = 0; k < step; k++){
            curr += mData[i+k]/step;
        }
        double next = 0;
        for(unsigned k = 0; k < step; k++){
            next += mData[i+step+k]/step;
        }

        double slope = (next - curr)/step;
        //std::cout << curr << ", " << next << ", " << slope << std::endl;

        if(slope > 0){
            counterPositive++;
        }
        if(counterPositive>4){
            if(slope < 0){
                // Found max!
                mLEtmax1->setText(QString::number(mTs[i+step/2]-mTs[0],'g',5));
                mLEvmax1->setText(QString::number(mData[i+step/2]-mData[0],'g',5));
            }
        }
    }
}