//
//
//
//
//

#ifndef ANALYSISWINDOW_H_
#define ANALYSISWINDOW_H_

#include <ros/ros.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialog>
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include "qcustomplot.h"

class AnalisysWindow : public QDialog {
    Q_OBJECT
    public:
        AnalisysWindow(std::vector<float> _refs, 
                                std::vector<float> _data, 
                                std::vector<float> _ts, 
                                std::vector<float> _keyTs, 
                                QWidget *parent = 0);
        virtual ~AnalisysWindow(){};

    private:
        void buildPanel();
        void buildGraph();
    private slots:
        void computeTsig();
        void computeAvgPer();
        void computeMax1();
    private:
        std::vector<float> mRefs, mData, mTs, mKeyTs;

        QWidget     *mCentralWidget;
        QHBoxLayout *mMainLayout;
        QVBoxLayout *mPanel;
        QCustomPlot *mGraph;

        QLineEdit *mLEt0, *mLEt1, *mLEt2;
        QLineEdit *mLEtsig, *mLEvmax1, *mLEtmax1, *mLEavgPer;
        QPushButton *mButtsig, *mButvmax1, *mButtmax1, *mButavgPer;
        QHBoxLayout *mLayTs, *mLaytsig, *mLayvmax1, *mLaytmax1, *mLayavgPer;

};

#endif