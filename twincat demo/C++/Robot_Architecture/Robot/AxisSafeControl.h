/** @file　AxisSafeControl.h
 *  @brief class of safety servo function
 *  @author wangyuxing
 *  @date 2020
 *  @version 0.1.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#ifndef AXISSAFECONTROL_H
#define AXISSAFECONTROL_H
#include "AxisControl.h"
namespace DucoCobot {

class AxisSafeControl:public AxisControl
{
public:
    AxisSafeControl(EcMaster* axis_link, const double& cycle_time);

    ~AxisSafeControl(){}
public:
    /**
     * @brief inputUpdate
     */
    void inputUpdate() override;
    /**
     * @brief setTorqueBand: axis output torque range setter interface
     * @param low_limit: low limit value of torque range
     * @param high_limit: high limit value of torque range
     */
    void setTorqueBand(double& low_limit, double& high_limit) override;
    /**
     * @brief getCurrent
     * @return
     */
    double getCurrent() const override;
    /**
     * @brief servoParamListDownload: download current servo parmeters list into servo
     * @return EC error type
     */
    /**
     * @brief servoParamListDownload: download current servo parmeters list into servo
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param bDone: process finish return signal
     * @return EC error type
     */
    unsigned int  servoParamListDownload() override;
    /**
     * @brief getServoSoftwareVersion: servo software version read interfaces
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param software_version: servo software version return value
     * @param bDone: process finish return signal
     * @return EC error type
     */
    unsigned int  getServoVersion( unsigned char* software_version) override;
    /**
     * @brief getServoMonitoringCode: servo monitoring code read interfaces, only valid when servo error is detected
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param monitoring_code: servo monitoring code return value
     * @param bDone: process finish return signal
     * @return EC error type
     */
    unsigned int  getServoMonitoringCode(double* monitoring_code) override;
    /**
     * @brief setBrakeMode; servo brake operation mode setter interface
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param tar_mode: target servo brake operation mode
     * @param dDone: process finish return signal
     * @return EC error type
     */
    unsigned int  setBrakeMode(BrakeMode tar_mode) override;

    /**
     * @brief getServoErrMsg: servo error message getter interface
     * @param err_msg: return servo error message
     */
    void getServoErrMsg(char * err_msg) override;
    /**
     * the following interfaces are single servo parameters download/upload protocals
     */
    /**
     * @brief single servo config parameter download interface
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param id: reference to single servo config parameter id
     * @param qFmt: reference to single servo config parameter qFmt
     * @param value: reference to single servo config parameter value
     * @param bDone: process download finish signal
     * @return unsigned int : return EC error
     */
    unsigned int  servoSingleParamDownload(const int& id, const int& qFmt, const double &value) override;
    /**
     * @brief single servo config parameter upload interface
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param id: reference to single servo config parameter id
     * @param qFmt: reference to single servo config parameter qFmt
     * @param value: reference to single servo config parameter value
     * @param bDone: process download finish signal
     * @return unsigned int : return EC error
     */
    unsigned int  servoSingleParamUpload(const int &id, const int &qFmt, double& value) override;

protected:
    EcMaster* _axis_safe_link=nullptr;
};
}


#endif
