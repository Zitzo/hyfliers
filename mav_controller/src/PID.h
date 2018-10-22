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

#ifndef PID_H_
#define PID_H_

#include <limits>
#include <ros/ros.h>
//#include <mav_controller/Float32Param.h>
#include <thread>

class PID
{
  public:
    PID(float _kp, float _ki, float _kd,
        float _minSat = std::numeric_limits<float>::min(),
        float _maxSat = std::numeric_limits<float>::max(),
        float _minWind = std::numeric_limits<float>::min(),
        float _maxWind = std::numeric_limits<float>::max());
    float update(float _val, float _incT);

    bool enableRosInterface(std::string _tag);

    float reference() { return mReference; }
    void reference(float _ref)
    {
        mReference = _ref;
        mAccumErr = 0;
        mLastError = 0;
        mLastResult = 0;
        mBouncingFactor = 0.1;
    }

    void reset()
    {
        mAccumErr = 0;
        mLastError = 0;
        mLastResult = 0;
        mBouncingFactor = 0.1;
    }

    float kp() const { return mKp; }
    float ki() const { return mKi; }
    float kd() const { return mKd; }

    void kp(float _kp) { mKp = _kp; }
    void ki(float _ki) { mKi = _ki; }
    void kd(float _kd) { mKd = _kd; }

    void setSaturations(float _min, float _max)
    {
        mMinSat = _min;
        mMaxSat = _max;
    }
    void getSaturations(float _min, float _max)
    {
        _min = mMinSat;
        _max = mMaxSat;
    }

    void setWindupTerms(float _min, float _max)
    {
        mWindupMin = _min;
        mWindupMax = _max;
    }
    void getWindupTerms(float _min, float _max)
    {
        _min = mWindupMin;
        _max = mWindupMax;
    }

  private:
    /*bool serviceKp(mav_controller::Float32Param::Request &_req, mav_controller::Float32Param::Response &_res) { mKp = _req.param; };
    bool serviceKi(mav_controller::Float32Param::Request &_req, mav_controller::Float32Param::Response &_res) { mKi = _req.param; };
    bool serviceKd(mav_controller::Float32Param::Request &_req, mav_controller::Float32Param::Response &_res) { mKd = _req.param; };
    bool serviceKsat(mav_controller::Float32Param::Request &_req, mav_controller::Float32Param::Response &_res)
    {
        mMinSat = -_req.param;
        mMaxSat = _req.param;
    };
    bool serviceKwindup(mav_controller::Float32Param::Request &_req, mav_controller::Float32Param::Response &_res)
    {
        mWindupMin = -_req.param;
        mWindupMax = _req.param;
    };*/

  private:
    float mReference;
    float mKp, mKi, mKd;
    float mMinSat, mMaxSat;
    float mWindupMin, mWindupMax;
    float mLastResult, mLastError, mAccumErr;
    ros::ServiceServer mServiceKp, mServiceKi, mServiceKd, mServiceSat, mServiceWind;
    ros::Publisher mPubKp, mPubKi, mPubKd, mPubSat, mPubWind;
    double mBouncingFactor = 0.1;
    std::thread mParamPubThread;
};

#endif
