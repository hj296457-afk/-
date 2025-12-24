/** @file　TcpControl.cpp
 *  @brief tcp board function class
 *  @author wangyuxing
 *  @date 2021
 *  @version 0.1.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#include <string.h>
#include "TcpControl.h"
#include  <memory>
#include <unistd.h>

using namespace std;
namespace  DucoCobot {

void TcpControl::setTcpDigitalOut(bool *digital_out)
{
    memcpy(digital_out_,digital_out,sizeof (digital_out_));
}

void TcpControl::getTcpDigitalOutWrite(bool *digital_out_write)
{
    memcpy(digital_out_write,digital_out_,sizeof (digital_out_));
}
void TcpControl::getTcpDigitalIn(bool *digital_in)
{
    memcpy(digital_in,digital_in_,sizeof (digital_in_));
}
void TcpControl::setLEDLight(const LEDLight &led_light)
{
    led_light_=led_light;
}

bool TcpControl::getTcpButtonT()
{
    return tcp_button_T_;
}

bool TcpControl::getTcpButtonS()
{
    return tcp_button_S_;
}

EcMaster::SerialData TcpControl::getRS485In()
{
    return tcp_pdo_inputs_.RS485Inputs;
}

void TcpControl::setRS485Out(EcMaster::SerialData rs485_out)
{
    tcp_pdo_outputs_.RS485Outputs = rs485_out;
}

void TcpControl::getAnalogVoltageInputs(double *analog_voltage_inputs)
{
    double analog_voltage[2];
    analog_voltage[0] = tcp_pdo_inputs_.AnalogVoltageInputs[0]/1000.0;
    analog_voltage[1] = tcp_pdo_inputs_.AnalogVoltageInputs[1]/1000.0;
    memcpy(analog_voltage_inputs,analog_voltage,sizeof (analog_voltage));
}

double TcpControl::getTemperature()
{
    return tcp_pdo_inputs_.TemperatureValue/100.0;
}

void TcpControl::getAccelerationMg(int16_t *acceleration_mg)
{
    memcpy(acceleration_mg,tcp_pdo_inputs_.AccelerationMg,sizeof (tcp_pdo_inputs_.AccelerationMg));
}

uint32_t TcpControl::getErrId()
{
    errid = tcp_pdo_inputs_.ErrorCode;
    return (errid==0) ? 0 : 0x13040000 + static_cast<uint32_t>(errid);
}

void TcpControl::getErrMsg(char *main_msg, char *detailed_msg)
{
    strcpy(main_msg,main_err_msg_);
    strcpy(detailed_msg,detailed_err_msg_);
}

unsigned int  TcpControl::setRS485BaudRate(BaudRate &baud_rate)
{
    if (tcpboard_link_&& tcpboard_link_->notSimulation())
    {
        unsigned int  ret = 0;
        ret=tcpboard_link_->sdoDownload(this->slave_id, 0x8000,0x03,reinterpret_cast<unsigned char *>(& baud_rate),sizeof(unsigned int),SDOTIMEOUT);
        if(ret!=0)
            return ret;
    }
    return 0;
}

unsigned int  TcpControl::setDigitalOutputsMode(DigitalOutputMode *digital_output_mode)
{
    if (tcpboard_link_&& tcpboard_link_->notSimulation())
    {
        unsigned short  sdo_buffer = (static_cast<unsigned short>(digital_output_mode[1])<<1)+static_cast<unsigned short>(digital_output_mode[0]);
        unsigned int  ret = 0;
        ret=tcpboard_link_->sdoDownload(slave_id,0x8000,0x04,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(unsigned int), SDOTIMEOUT);
        if(ret!=0)
            return ret;
    }
    return 0;
}

unsigned int  TcpControl::getDigitalOutputsMode(TcpControl::DigitalOutputMode *digital_output_mode)
{
    if (tcpboard_link_&& tcpboard_link_->notSimulation())
    {
        unsigned short  sdo_buffer = 0;
        unsigned int  ret = 0;
        int recv_data=0;
        ret = tcpboard_link_->sdoUpload(slave_id,0x8000,0x04,reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(int),&recv_data,SDOTIMEOUT);
        if(ret!=0)
            return ret;
        digital_output_mode[0] = static_cast<DigitalOutputMode>(sdo_buffer & 0x01);
        digital_output_mode[1] = static_cast<DigitalOutputMode>(sdo_buffer & 0x02);
    }
    return 0;
}

unsigned int  TcpControl::setLEDLightBrightness(unsigned int &brightness)
{
    if (tcpboard_link_&& tcpboard_link_->notSimulation())
    {
        unsigned int  ret = 0;
        ret=tcpboard_link_->sdoDownload(slave_id,0x8000,0x08,reinterpret_cast<unsigned char *>(& brightness),sizeof(int),SDOTIMEOUT);
        if(ret!=0)
            return ret;
    }
    return 0;
}

unsigned int  TcpControl::setReuseInterface(ReuseInterface &reuse_interface)
{
    if (tcpboard_link_&& tcpboard_link_->notSimulation())
    {
        unsigned int  ret = 0;
        ret=tcpboard_link_->sdoDownload(slave_id,0x8000,0x05,reinterpret_cast<unsigned char *>(& reuse_interface),sizeof(int),SDOTIMEOUT);
        if(ret!=0)
            return ret;
    }
    return 0;
}

unsigned int  TcpControl::getSoftwareVersion(char *software_version)
{
    software_version[0]=0;
    software_version[1]=0;
    software_version[2]=0;
    if (tcpboard_link_&& tcpboard_link_->notSimulation())
    {
        unsigned int  ret = 0;
        unsigned int  sdo_buffer;
        int recv_data;
        tcpboard_link_->sdoUpload(slave_id,0x8000, 0x02, reinterpret_cast<unsigned char *>(& sdo_buffer),sizeof(int),&recv_data,SDOTIMEOUT);
        software_version[0] = char((sdo_buffer>>11) & 0x1F);
        software_version[1] = char((sdo_buffer>>4) & 0x7F);
        software_version[2] = char(sdo_buffer & 0x0F);
    }
    return 0;
}
}

