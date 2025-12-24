/** @file　AxisSafeControl.cpp
 *  @brief class of safe5in1 servo function
 *  @author wangyuxing
 *  @date 2021
 *  @version 0.1.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#include "AxisSafeControl.h"
#include "EcMaster/EcMaster.h"
#include "memory.h"
#include <iostream>
namespace DucoCobot{

AxisSafeControl::AxisSafeControl(EcMaster *axis_link, const double &cycle_time):
    AxisControl(axis_link,cycle_time)
{
    _axis_safe_link = static_cast<EcMaster*>(axis_link);
}

void AxisSafeControl::inputUpdate()
{
    NCAxis::inputUpdate();
    axis_pdo_inputs_ = _axis_safe_link->getAxisPdoInputs(this->axis_num);
}

void AxisSafeControl::setTorqueBand(double &low_limit, double &high_limit)
{
    if (!_axis_safe_link || !_axis_safe_link->notSimulation())
        return;

    torque_low_lim_ = MIN_SERVO_CURRENT_RATE;
    torque_high_lim_ = MAX_SERVO_CURRENT_RATE;

    if (!bsimulation_)
    {
        if (motor_param_.invert)
        {
            _axis_safe_link->axis_pdo_outputs[this->axis_num].TorqueBand=(uint32_t(-torque_high_lim_) & 0x0000ffff)+ (uint32_t(-torque_low_lim_)<<16);
        }
        else
        {
            _axis_safe_link->axis_pdo_outputs[this->axis_num].TorqueBand=(uint32_t(torque_low_lim_) & 0x0000ffff)+ (uint32_t(torque_high_lim_)<<16);
        }
    }
    return;
}

double AxisSafeControl::getCurrent() const
{
    return axis_pdo_inputs_.ActualTorque;
}

void AxisSafeControl::getServoErrMsg(char *err_msg)
{
    unsigned int err_index = 0;
    for (unsigned int i=0;i<8*sizeof (servo_errid_);++i)
    {
        if ((servo_errid_>>i)&0x01)
        {
            err_index =  i+1;
        }
    }
    strcpy(err_msg_,"");
    switch (err_index)
    {
    case 1:
        strcpy(err_msg_,"Under voltage soft");
        break;
    case 2:
        strcpy(err_msg_,"Over voltage soft");
        break;
    case 3:
        strcpy(err_msg_,"Over current soft");
        break;
    case 4:
        strcpy(err_msg_,"Over speed");
        break;
    case 5:
        strcpy(err_msg_,"IGBT temperature sensor error");
        break;
    case 6:
        strcpy(err_msg_,"Hardware error");
        break;
    case 7:
        strcpy(err_msg_,"Position error over");
        break;
    case 8:
        strcpy(err_msg_,"IGBT over temperature");
        break;
    case 9:
        strcpy(err_msg_,"Self test fault");
        break;
    case 10:
        strcpy(err_msg_,"No motion command");
        break;
    case 11:
        strcpy(err_msg_,"Current u sensor error");
        break;
    case 12:
        strcpy(err_msg_,"ABS data invalid");
        break;
    case 13:
        strcpy(err_msg_,"Voltage sensor error");
        break;
    case 14:
        strcpy(err_msg_,"ABS lost connect");
        break;
    case 15:
        strcpy(err_msg_,"Current sample module error");
        break;
    case 16:
        strcpy(err_msg_,"Saturation");
        break;
    case 17:
        strcpy(err_msg_,"Hardware version error");
        break;
    case 18:
        strcpy(err_msg_,"Servo program stucked");
        break;
    case 19:
        strcpy(err_msg_,"Motor over heat");
        break;
    case 20:
        strcpy(err_msg_,"Current sum non-zero");
        break;
    case 21:
        strcpy(err_msg_,"DSP over temperature");
        break;
    case 22:
        strcpy(err_msg_,"Encoder hall cross check");
        break;
    case 23:
        strcpy(err_msg_,"ABS CRC error");
        break;
    case 24:
        strcpy(err_msg_,"Phase reverse");
        break;
    case 25:
        strcpy(err_msg_,"ABS data warning");
        break;
    case 26:
        strcpy(err_msg_,"INC error");
        break;
    case 27:
        strcpy(err_msg_,"Hall-less low");
        break;
    case 28:
        strcpy(err_msg_,"Current v sensor error");
        break;
    case 29:
        strcpy(err_msg_,"Current w sensor error");
        break;
    case 30:
        strcpy(err_msg_,"Hallless double check error");
        break;
    case 31:
        strcpy(err_msg_,"INC no Z");
        break;
    case 32:
        strcpy(err_msg_,"Brake over current");
        break;
    default:
    {
        if (0x60 == nc_axis_errid_)
        {
            strcpy(err_msg_,"Abnormal FSA state machine transition");
        }
        else if (0x61 == nc_axis_errid_)
        {
            strcpy(err_msg_,"Illegal FSA events command");
        }
        else if (0x62 == nc_axis_errid_)
        {
            strcpy(err_msg_,"Position error exceed NC limitation");
        }
        break;
    }
        break;
    }
    strcpy(err_msg,err_msg_);
}

unsigned int  AxisSafeControl::servoParamListDownload()
{
    unsigned int  ret = 0;
    if (!_axis_safe_link || !_axis_safe_link->notSimulation() || (servo_param_list_.size() == 0))
    {
        return 0;
    }

    for(int i=0;i<this->servo_param_list_.size();i++)
    {
        if(0!=servoSingleParamDownload(servo_param_list_[i].id,servo_param_list_[i].qFmt,servo_param_list_[i].value))
        {
            return -1;
        }
    }
    if(0!=servoSingleParamDownload(2,0,1))
        return -1;
    return 0;
}

unsigned int  AxisSafeControl::getServoVersion(unsigned char *software_version)
{
    if (!_axis_safe_link || !_axis_safe_link->notSimulation())
    {
        software_version[0] = 0;
        software_version[1] = 0;
        software_version[2] = 0;
        return 0;
    }

    unsigned int  ret = 0;
    unsigned int  sdo_buffer = 0;

    sdo_buffer = (0x01<<24)+(0x04<<16)+(0x63<<8)+0x00;
    ret = axis_link_->sdoDownload(this->slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);

    int recv_len;
    //read servo version info
    ret = axis_link_->sdoUpload(this->slave_id,sdo_r_index_,sdo_r_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),&recv_len,SDOTIMEOUT);
    software_version_[0] = static_cast<unsigned char>((sdo_buffer>>11) & 0x1F);
    software_version_[1] = static_cast<unsigned char>((sdo_buffer>>4) & 0x7F);
    software_version_[2] = static_cast<unsigned char>((sdo_buffer) & 0x0F);

    memcpy(software_version,software_version_,sizeof (software_version_));
    return 0;

}

unsigned int  AxisSafeControl::getServoMonitoringCode(double *monitoring_code)
{
    if (!_axis_safe_link || !_axis_safe_link->notSimulation())
    {
        for (unsigned int i=0;i<15;++i)
        {
            monitoring_code_[i] = 0;
        }
        memcpy(monitoring_code,monitoring_code_,sizeof (monitoring_code_));
        return 0;
    }

    unsigned int  ret=0;
    for(int i=0;i<15;i++)
    {
        ret=servoSingleParamDownload(63,0,monitoring_id_[monitor_code_cnt]);
        ret = servoSingleParamUpload(62,0,monitoring_code_[monitor_code_cnt]);

    }
    ret = servoSingleParamUpload(75,6,monitoring_code_[15]);
    memcpy(monitoring_code,monitoring_code_,sizeof (monitoring_code_));
    return ret;
}

unsigned int AxisSafeControl::setBrakeMode(BrakeMode tar_mode)
{
    if (!_axis_safe_link || !_axis_safe_link->notSimulation())
    {
        return 0;
    }
    unsigned int  ret = 0;
    double value = 1;


    if (tar_mode == BrakeMode::active)
        value = 0;
    else if (tar_mode == BrakeMode::normal)
        value = 1;
    else if (tar_mode == BrakeMode::deactive)
        value = 65535;
    ret = servoSingleParamDownload(60,0,value);

    return ret;

}

unsigned int  AxisSafeControl::servoSingleParamDownload(const int &id, const int &qFmt,const double &value)
{
    int ret=0;
    unsigned int sdo_buffer=0;
    sdo_buffer = (0x02<<24)+(0x01<<16)+(0x63<<8)+0x01;
    ret=axis_link_->sdoDownload(slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    if(ret!=0)
        return ret;
    usleep(10000);
    if (value < 1e-8f)
        sdo_buffer = (0x01<<16)+static_cast<unsigned int>(fabs(static_cast<double>(value))*pow(2,qFmt));
    else
        sdo_buffer = (0x00<<16)+static_cast<unsigned int>(fabs(static_cast<double>(value))*pow(2,qFmt));
    ret=axis_link_->sdoDownload(slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    if(ret!=0)
        return ret;
    usleep(10000);
    sdo_buffer = (0x01<<16)+static_cast<unsigned int>(id);
    ret=axis_link_->sdoDownload(slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    if(ret!=0)
        return ret;
    usleep(10000);
    return 0;
}

unsigned int  AxisSafeControl::servoSingleParamUpload(const int &id, const int &qFmt, double &value)
{
    if (!_axis_safe_link || !_axis_safe_link->notSimulation())
    {
        value = 0;
        return 0;
    }

    unsigned int  ret = 0;
    unsigned int  sdo_buffer = 0;
    int recv_len;


    //write request parameter info header
    sdo_buffer = (0x02<<24)+(0x04<<16)+(0x63<<8)+0x01;
    ret = axis_link_->sdoDownload(this->slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    usleep(20);


    sdo_buffer = (static_cast<unsigned int >(100+id))<<8;
    ret = axis_link_->sdoDownload(this->slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    usleep(20);


    sdo_buffer = 0x02<<16;
    ret = axis_link_->sdoDownload(this->slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    usleep(20);

    sdo_buffer = (0x07<<24)+(0x04<<16)+(0x63<<8)+0x00;
    ret = axis_link_->sdoDownload(this->slave_id,sdo_w_index_,sdo_w_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),SDOTIMEOUT);
    usleep(20);

    ret = axis_link_->sdoUpload(this->slave_id,sdo_r_index_,sdo_r_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),&recv_len,SDOTIMEOUT);
    usleep(20);

    value = ((sdo_buffer>>20)&0x01)==1 ? -1.0 : 1.0;
    value = value*static_cast<double>(static_cast<int>(sdo_buffer)/pow(2,qFmt));

    ret = axis_link_->sdoUpload(this->slave_id,sdo_r_index_,sdo_r_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),&recv_len,SDOTIMEOUT);
    usleep(20);

    ret =axis_link_->sdoUpload(this->slave_id,sdo_r_index_,sdo_r_subindex_,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(sdo_buffer),&recv_len,SDOTIMEOUT);
    usleep(20);
    return 0;


}
}

