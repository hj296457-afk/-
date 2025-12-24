#ifndef ECMASTER_H
#define ECMASTER_H
#include "Global.h"
namespace DucoCobot {
class Robot;
/**
 * @brief  ec master abstract
 *
 */
class EcMaster
{
#define FOEMAX  3072*1024
#define SDOTIMEOUT 600000
public:
    /**
     * @brief pdo input struct for axis
     *
     */
    typedef struct
    {
        int32_t ActualPosition = 0;
        int32_t ActualVelocity = 0;
        int16_t ActualTorque = 0;
        int32_t AuxiliaryPosition = 0;
        int32_t AuxiliaryVelocity = 0;
        int16_t AuxiliaryTorque = 0;
        int8_t  ModeOfOperationDisplay = 0;
        uint16_t Statusword = 0;
        uint32_t DigitalInputs = 0;
        uint32_t ErrorCode = 0;
    }AxisPdoInputs;

    /**
     * @brief pdo output struct for axis
     *
     */
    typedef struct
    {
        int32_t TargetPosition = 0;
        int32_t TargetVelocity = 0;
        int16_t TargetTorque = 0;
        int8_t ModeOfOperation = 0;
        uint16_t Controlword = 0;
        uint16_t DigitalOutputs = 0;
        uint8_t SafetyBoardState = 0;
        int32_t TorqueBand = 0;

    }AxisPdoOutputs;

    typedef struct
    {
        uint16_t count = 0;
        uint16_t length = 0;
        uint8_t buffer[32];
    } SerialData;


    /**
     * @brief pdo input struct for tcp board
     *
     */
    typedef struct
    {
        uint8_t ErrorCode;
        uint8_t DigitalInputs;
        uint16_t AnalogVoltageInputs[2];
        int16_t TemperatureValue;
        int16_t AccelerationMg[3];
        SerialData RS485Inputs;

    }TcpPdoInputs;

    /**
      * pdo output struct for tcp board
      */
    typedef struct
    {
        uint8_t LEDWorkControl;
        uint8_t DigitalOutputsControl;
        SerialData RS485Outputs;
    }TcpPdoOutputs;

    /**
     * @brief master init
     *
     * @return int
     */
    virtual int init()=0;

    /**
     * @brief config master
     *
     * @return int
     */
    virtual int configMaster()=0;

    /**
     * @brief foe file upload
     *
     * @param slave_id Slave ID
     * @param file_name File name of slave file to read.
     * @param file_name_len Length of slave file name in bytes.
     * @param data file Buffer receiving transfered data
     * @param data_len Buffer length [bytes]
     * @param recv_len Length of received data [byte]
     * @param timeout
     * @return int
     */
    virtual int foeFileUpload(int slave_id, char *file_name, int file_name_len, char *data,int data_len, int *recv_len, int timeout)=0;

    /**
     * @brief sdo download
     *
     * @param slave_id Slave ID
     * @param index Object index.
     * @param sub_index Object sub index.
     * @param data Buffer containing transfered data
     * @param data_len Buffer length [bytes]
     * @param timeout
     * @return int
     */
    virtual int sdoDownload(int slave_id,int index,int sub_index,unsigned char* data,int data_len,int timeout)=0;

    /**
     * @brief
     *
     * @param slave_id Slave ID
     * @param index Object index.
     * @param sub_index Object sub index.
     * @param data Buffer receiving transfered data
     * @param data_len Buffer length [bytes]
     * @param recv_len Length of received data [byte]
     * @param timeout
     * @return int
     */
    virtual int sdoUpload(int slave_id, int index, int sub_index,unsigned char *data, int data_len, int* recv_len,int timeout)=0;

    /**
     * @brief Assign the ethercat bus input to pdo input data
     *
     * @return int
     */
    virtual int readFromBus()=0;

    /**
     * @brief pdo output data is assigned to the ethercat bus output
     *
     * @return int
     */
    virtual int writeToBus()=0;

    /**
     * @brief set master state(pre_op/op)
     *
     * @param ec_state
     * @return int
     */
    virtual int setMasterState(int ec_state)=0;

    /**
     * @brief get master state
     *
     * @return int
     */
    virtual int getMasterState()=0;

    /**
     * @brief set axis pdo output data
     *
     * @param pdo_outputs
     * @param num axis num
     */
    void setAxisPdoOutputs(const AxisPdoOutputs& pdo_outputs,int num)
    {
        axis_pdo_outputs[num]=pdo_outputs;
    }
    /**
     * @brief get axis pdo input data
     *
     * @param num axis num
     * @return AxisPdoInputs
     */
    AxisPdoInputs getAxisPdoInputs(int num)
    {
        return axis_pdo_inputs[num];
    }

    /**
     * @brief set tcp board pdo output data
     *
     * @param tcp_pdo
     */
    void setTcpPdoOutput(const TcpPdoOutputs &tcp_pdo)
    {
        this->tcp_pdo_output=tcp_pdo;
    }

    /**
     * @brief  get tcp board pdo input data
     *
     * @return TcpPdoInputs
     */
    TcpPdoInputs getTcpPdoInput()
    {
        return this->tcp_pdo_input;
    }

    /**
     * @brief is simulation mode?
     *
     * @return bool
     */
    bool notSimulation()
    {
        return !simulation;
    }
    /**
     * @brief set simulation mode
     *
     * @param sim
     */
    void setSimulation(bool sim)
    {
        simulation=sim;
    }

    AxisPdoInputs axis_pdo_inputs[AXIS_NUM]; /**< pdo input data for axis */
    AxisPdoOutputs axis_pdo_outputs[AXIS_NUM]; /**< pdo output data for axis*/
    TcpPdoInputs tcp_pdo_input; /**< pdo input data for tcp */
    TcpPdoOutputs tcp_pdo_output; /**< pdo output data for tcp */
    bool simulation=false; /**< Whether it is simulation mode*/

};

}



#endif
