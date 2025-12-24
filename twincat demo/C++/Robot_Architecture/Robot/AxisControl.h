/** @file　AxisControl.h
 *  @brief class of servo function
 *  @author wangyuxing
 *  @date 2020
 *  @version 0.1.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#ifndef AXISCONTROL_H
#define AXISCONTROL_H
#include "NCAxis.h"
#define MAX_SERVO_CURRENT_RATE 2000
#define MIN_SERVO_CURRENT_RATE -2000
#define RC_SERVO_COM_LAG 7

namespace s {
namespace mbl {

class Filter;

}
}

namespace DucoCobot
{

class AxisControl:public NCAxis
{
public:
    AxisControl(EcMaster* axis_link, const double& cycle_time):
        NCAxis(axis_link,cycle_time){}

    ~AxisControl(){}
public:

    /**
     * @brief setTorqueBand: axis output torque range setter interface
     * @param low_limit: low limit value of torque range
     * @param high_limit: high limit value of torque range
     */
    virtual void setTorqueBand(double& low_limit, double& high_limit) = 0;
    /**
     * @brief setServoParamList: servo config parameters list setter interface
     * @param servo_param_list: servo config parameters list
     */
    virtual void setServoParamList(const std::vector<ServoParam>& servo_param_list)
    {
        servo_param_list_ = servo_param_list;
    }
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
    virtual unsigned int  servoParamListDownload() = 0;
    /**
     * @brief getServoSoftwareVersion: servo software version read interfaces
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param software_version: servo software version return value
     * @param bDone: process finish return signal
     * @return EC error type
     */
    virtual unsigned int  getServoVersion( unsigned char* software_version) = 0;
    /**
     * @brief getServoMonitoringCode: servo monitoring code read interfaces, only valid when servo error is detected
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param monitoring_code: servo monitoring code return value
     * @param bDone: process finish return signal
     * @return EC error type
     */
    virtual unsigned int  getServoMonitoringCode( double* monitoring_code) = 0;
    /**
     * @brief setBrakeMode; servo brake operation mode setter interface
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param tar_mode: target servo brake operation mode
     * @param dDone: process finish return signal
     * @return EC error type
     */
    virtual unsigned int  setBrakeMode( BrakeMode tar_mode) = 0;

    /**
     * @brief getServoErrMsg: servo error message getter interface
     * @param err_msg: return servo error message
     */
    virtual void getServoErrMsg(char * err_msg) = 0;
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

    virtual unsigned int  servoSingleParamDownload(const int& id, const int& qFmt,const double &value) =0;
    /**
     * @brief single servo config parameter upload interface
     * @param bExec: execute signal, input true to activate function ,input false to reset function
     * @param id: reference to single servo config parameter id
     * @param qFmt: reference to single servo config parameter qFmt
     * @param value: reference to single servo config parameter value
     * @param bDone: process download finish signal
     * @return unsigned int : return EC error
     */
    virtual unsigned int  servoSingleParamUpload( const int& id, const int& qFmt, double& value) = 0;

protected:
    double torque_high_lim_ = 0, torque_low_lim_ = 0; /**< local torque range limits */
    std::vector<AxisControl::ServoParam> servo_param_list_; /**< local servo config parameters list */
    int monitoring_id_[15]={23,24,25,26,27,28,29,30,31,32,21,11,33,17,3};  /**< specifed auxiliary error monitoring code index */
    double monitoring_code_[16];  /**< local servo auxiliary error monitoring code array */
    unsigned char  param_version_[3];  /**< local servo config parameters list version */
    unsigned char software_version_[3];
    unsigned short sdo_w_index_ = 0x2020;  /**< local servo sdo write channel index */
    unsigned char sdo_w_subindex_ = 0;  /**< local servo sdo write channel sub-index */
    unsigned short sdo_r_index_ = 0x2030;  /**< local servo sdo read channel index */
    unsigned char  sdo_r_subindex_ = 0;  /**< local servo sdo read channel sub_index */
    unsigned int version_up_state = 0, monitor_code_up_state = 0, param_dw_state = 0, param_up_state = 0, list_dw_state = 0, brake_set_state = 0, peak_voltage_state = 0, valley_voltage_state = 0;  /**< local sdo&foe function process sub state machine */
    unsigned int monitor_code_cnt = 0, list_dw_cnt = 0;    /**< local counter */
    s::mbl::Filter* _min_cmd_filter;
    s::mbl::Filter* _max_cmd_filter;
};
}

#endif
