//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include "PID.h"
#include <algorithm>
//#include <std_msgs/Float32.h>
#include <thread>
#include <chrono>

PID::PID(float _kp, float _ki, float _kd, float _minSat, float _maxSat, float _minWind, float _maxWind)
{
    mKp = _kp;
    mKi = _ki;
    mKd = _kd;
    mMinSat = _minSat;
    mMaxSat = _maxSat;
    mWindupMin = _minWind;
    mWindupMax = _maxWind;
}

bool PID::enableRosInterface(std::string _tag)
{
    if (ros::isInitialized())
    {
        /*ros::NodeHandle np;

        mServiceKp = np.advertiseService(_tag + "/kp", &PID::serviceKp, this);
        mServiceKi = np.advertiseService(_tag + "/ki", &PID::serviceKi, this);
        mServiceKd = np.advertiseService(_tag + "/kd", &PID::serviceKd, this);
        mServiceSat = np.advertiseService(_tag + "/saturation", &PID::serviceKsat, this);
        mServiceWind = np.advertiseService(_tag + "/windup", &PID::serviceKwindup, this);

        mPubKp = np.advertise<std_msgs::Float32>(_tag + "/kp", 1);
        mPubKi = np.advertise<std_msgs::Float32>(_tag + "/ki", 1);
        mPubKd = np.advertise<std_msgs::Float32>(_tag + "/kd", 1);
        mPubSat = np.advertise<std_msgs::Float32>(_tag + "/saturation", 1);
        mPubWind = np.advertise<std_msgs::Float32>(_tag + "/windup", 1);*/

        mParamPubThread = std::thread([&]() {
            while (ros::ok())
            {/*
                std_msgs::Float32 param;

                param.data = mKp;
                mPubKp.publish(param);
                param.data = mKi;
                mPubKi.publish(param);
                param.data = mKd;
                mPubKd.publish(param);
                param.data = mMaxSat;
                mPubSat.publish(param);
                param.data = mWindupMax;
                mPubWind.publish(param);
                std::this_thread::sleep_for(std::chrono::seconds(3));*/
            }
        });
        return true;
    }
    else
    {
        return false;
    }
}

float PID::update(float _val, float _incT)
{
    float dt = _incT; // 666 input arg?

    float err = mReference - _val;
    mAccumErr += err * dt;
    // Apply anti wind-up 777 Analyze other options
    mAccumErr = std::min(std::max(mAccumErr, mWindupMin), mWindupMax);

    // Compute PID
    mLastResult = mKp * err + mKi * mAccumErr + mKd * (err - mLastError) / dt;
    mLastError = err;

    // Saturate signal
    mLastResult = std::min(std::max(mLastResult, mMinSat), mMaxSat);
    mLastResult *= mBouncingFactor;
    mBouncingFactor *= 2.0;
    mBouncingFactor = mBouncingFactor > 1.0 ? 1.0 : mBouncingFactor;
    return mLastResult;
}
