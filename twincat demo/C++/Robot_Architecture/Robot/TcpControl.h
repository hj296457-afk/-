/** @file　TcpControl.h
 *  @brief safety board class
 *  @author wangyuxing
 *  @date 2020
 *  @version 0.1.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#ifndef TCPCONTROL_H
#define TCPCONTROL_H
#include "EcMaster/EcMaster.h"
#include <string>

namespace DucoCobot
{

class TcpControl
{
public:
    /**
     * @brief robot end-effector using led light type enum
     */
    typedef enum LEDLight{
        OFF = 0x00,
        BLUE_SLOW_BLINK = 0x01,
        BLUE_PULSE_BLINK = 0x02,
        RED_PULSE_BLINK = 0x03,
        RED_SLOW_BLINK = 0x04,
        GREEN_FAST_BLINK = 0x05,
        WHITE_DIM = 0x06,
        RED_FAST_BLINK = 0x07,
        RED = 0x08,
        GREEN = 0x09,
        BLUE = 0x0A,
        WHITE = 0x0B
    }LEDLight;

    typedef enum BaudRate
    {
        RATE_10k = 10,
        RATE_20k = 20,
        RATE_50k = 50,
        RATE_100k = 100,
        RATE_125k = 125,
        RATE_250k = 250,
        RATE_500k = 500,
        RATE_1000k = 1000,
        RATE_9600 = 9600,
        RATE_19200 = 19200,
        RATE_38400 = 38400,
        RATE_57600 = 57600,
        RATE_115200 = 115200
    } BaudRate;

    /**
      * @brief digital output mode enum
      */
    typedef enum DigitalOutputMode{
        PNP = 0,
        NPN = 1
    } DigitalOutputMode;
    /**
      * @brief communication interface input signal selection enum
      */
    typedef enum ReuseInterface{
        ANALOG_IN = 0,
        RS485 = 1
    }ReuseInterface;

public:
    TcpControl(EcMaster* tcpboard_link)
    {
        memset(&tcp_pdo_inputs_,0,sizeof(tcp_pdo_inputs_));
        memset(&tcp_pdo_outputs_,0,sizeof(tcp_pdo_outputs_));
        tcpboard_link_ = tcpboard_link;
        memset(digital_in_,0,sizeof (digital_in_));
        memset(digital_out_,0,sizeof (digital_out_));
        tcp_button_S_ = false;
        tcp_button_T_ = false;
        led_light_ = LEDLight::WHITE;
        errid = 0;
        memset(main_err_msg_,0,sizeof (main_err_msg_));
        memset(detailed_err_msg_,0,sizeof (detailed_err_msg_));
    }

    ~TcpControl(){}

    /**
     * @brief inputUpdate: tcp board online input data update,
     *        including pdo data unit tranfer and protocal online resolving.
     *        make sure this function is called at the start of one executing cycle
     */
    void inputUpdate()
    {
        if (tcpboard_link_&& tcpboard_link_->notSimulation())
        {
            tcp_pdo_inputs_ = tcpboard_link_->getTcpPdoInput();
        }

        digital_in_[0] = (tcp_pdo_inputs_.DigitalInputs & 0x01) ? true : false;
        digital_in_[1] = (tcp_pdo_inputs_.DigitalInputs & 0x02) ? true : false;
        tcp_button_T_ = (tcp_pdo_inputs_.DigitalInputs & 0x40) ? true : false;
        tcp_button_S_ = (tcp_pdo_inputs_.DigitalInputs & 0x80) ? true : false;

    }
    /**
     * @brief outputSend: tcp board online output data send,
     *        including pdo data unit transfer and protocal online resolving,
     *        make sure this function is called at the end of one executing cycle
     */
    void outputSend()
    {
        tcp_pdo_outputs_.LEDWorkControl = uint8_t(led_light_);
        tcp_pdo_outputs_.DigitalOutputsControl = 0x03 & ((digital_out_[1] << 1) | digital_out_[0]);

        if (tcpboard_link_&& tcpboard_link_->notSimulation())
        {
            tcpboard_link_->setTcpPdoOutput(tcp_pdo_outputs_);
        }
    }
    /**
     * @brief tcp board io output setter
     * @param io_out: pointer to io output vector
     */
    void setTcpDigitalOut(bool* digital_out);
    /**
     * @brief getTcpDigitalOutWrite: tcp board io output online value getter interface
     * @param io_out_write
     */
    void getTcpDigitalOutWrite(bool* digital_out_write);
    /**
     * @brief tcp io input getter
     * @param io_in: pointer to io input vector
     */
    void getTcpDigitalIn(bool* digital_in);
    /**
     * @brief tcp board led light type setter
     * @param led_light: reference to target led light type
     */
    void setLEDLight(const LEDLight& led_light = LEDLight::OFF);
    /**
     * @brief tcp board T button getter
     * @param tcp_button_t: pointer to t buttontcpboard_link_
     */
    bool getTcpButtonT();
    /**
     * @brief tcp board S button getter
     * @param tcp_button_s: pointer to s button
     */
    bool getTcpButtonS();
    /**
     * @brief rs485 original Txbuffer data getter
     * @param tx_buffer: pointer to tx buffer data vector
     */
    EcMaster::SerialData getRS485In();
    /**
     * @brief setRS485Out
     * @param rs485_out
     */
    void setRS485Out(EcMaster::SerialData rs485_out);
    /**
     * @brief getAnalogVoltageInputs: analog voltage input data getter interface, only valie when analog communication is selected
     * @param analog_voltage_inputs: return analog voltage input
     */
    void getAnalogVoltageInputs(double* analog_voltage_inputs);
    /**
     * @brief getTemperature: temperature value getter interface
     * @return temperature online value
     */
    double getTemperature();
    /**
     * @brief getAccelerationMg: acceleration mg value getter interface
     * @param acceleration_mg: return acceleration mg value
     */
    void getAccelerationMg(int16_t* acceleration_mg);
    /**
     * @brief getErrId: get tcp board error id
     * @return error id
     */
    uint32_t getErrId();
    /**
     * @brief getErrMsg: get tcp board error message
     * @param main_msg: return main error message
     * @param detailed_msg: return detailed error message
     */
    void getErrMsg(char* main_msg, char* detailed_msg);
    /**
     * @brief setRS485BaudRate: rs485 baud rate setter interface
     * @param baud_rate: target baud rate
     * @return ec error
     */
    unsigned int  setRS485BaudRate(BaudRate& baud_rate);
    /**
     * @brief setDigitalOutputsMode: digital output mode setter interface
     * @param digital_output_mode: target digital output mode
     * @return ec error
     */
    unsigned int  setDigitalOutputsMode(DigitalOutputMode* digital_output_mode);
    /**
     * @brief getDigitalOutputsMode: digital output mode setter interface
     * @param digital_output_mode: current digital output mode
     * @return ec error
     */
    unsigned int  getDigitalOutputsMode(DigitalOutputMode* digital_output_mode);
    /**
     * @brief setLEDLightBrightness: led light brightness setter interface
     * @param brightness: target led light brightness, range: 5-30
     * @return ec error
     */
    unsigned int  setLEDLightBrightness(unsigned int &brightness);
    /**
     * @brief setReuseInterface: reuse interface communication selection setter interface
     * @param reuse_interface: target communication selection
     * @return ec error
     */
    unsigned int  setReuseInterface(ReuseInterface& reuse_interface);
    /**
     * @brief getSoftwareVersion: software version getter interface
     * @param software_version: return software version
     * @return ec error
     */
    unsigned int  getSoftwareVersion(char* software_version);
    /**
     * @brief getTcpBoard: tcpboard object pointer getter interface
     * @return tcpboard object pointer
     */
    EcMaster* getTcpBoard(){return tcpboard_link_;}

    void setSlaveID(int id){this->slave_id=id;}

    int getSlaveID(){return this->slave_id;}
protected:
    EcMaster* tcpboard_link_;
    EcMaster::TcpPdoInputs tcp_pdo_inputs_;
    EcMaster::TcpPdoOutputs tcp_pdo_outputs_;

    uint8_t errid; /**< local error id */
    char main_err_msg_[64]; /**< local main error message */
    char detailed_err_msg_[256]; /**< local detailed error message */

    bool digital_out_[8]; /**< local io output */
    bool digital_in_[8]; /**< local io input */
    bool tcp_button_T_; /**< local board T button */
    bool tcp_button_S_; /**< local board S button */
    LEDLight led_light_; /**< local board led light type */
    int slave_id=0;
};
}
#endif
