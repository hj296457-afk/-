/** @file　NCAxis.h
 *  @brief class of robot axis supports CIA402 protocol
 *  @author wangyuxing
 *  @date 2020
 *  @version 0.1.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#ifndef NCAXIS_H
#define NCAXIS_H
#include "EcMaster/EcMaster.h"
#include "math.h"
#include <vector>
namespace DucoCobot {

const double rad2deg = 3.141592654/180;
const double deg2rad = 180/3.141592654;
const double PI = 3.141592654;


class EcMaster;

/**
 * @brief
 *
 */
class NCAxis
{
public:

    /**
     * @brief the following interfaces are servo application related interfaces
     *        including servo parameters operation, version info read, error analysis and software update
     */
    /**
     * @brief single servo config parameter data struct
     */
    struct ServoParam{
        std::string name = ""; /**< parameter name */
        int id = 0; /**< parameter id */
        int qFmt = 0; /**< parameter qFmt */
        float value = 0.0; /**< parameter value */
    };
    /**
     * @brief brake operation mode enum
     */
    enum BrakeMode{
        normal = 0,
        active = 1,
        deactive = 2
    };
    /**
     * @brief The BrakeParam struct
     */
    struct BrakeParam{
        double max_torque = 0;
        double qualified_torque = 0;
        double qualified_torque_factor = 0.8;
    };
    /**
     * @brief The EncoderType enum
     */
    enum EncoderType{
        ET_ABSOLUTE = 0,
        ET_INCREMENT
    };
    /**
     * @brief axis encoder parameters struct
     */
    typedef struct{
        EncoderType type = EncoderType::ET_ABSOLUTE;
        double scalefactor = 0; /**< scale factor from original count to rad in low-speed terminal of axis */
        double posbias = 0; /**< position bias of encoder in low-speed terminal of axis */
        bool invert = false; /**< invert flag */
    }EncoderParam;

    /**
     * @brief axis motor parameters struct
     */
    typedef struct{
        double inertia = 0; /**< motor rotor iniertia */
        double ratedcurrent = 0; /**< rated current, unit Amp */
        double maxcurrent = 0; /**< max current, unit Amp */
        double kt = 1;      /**< axis motor torque-current factor, unit Nm/Arms */
        bool invert = false; /**< invert flag */
        uint8_t pole_pair_num = 8;
        uint8_t stator_slots = 18;
    }MotorParam;

    /**
      * @brief axis self parameters struct
      */
    typedef struct{
        double ratio = 101;     /**< axis ratio */
        double rated_speed = 60;
        double rated_torque = 50;
        double max_torque = 50;
        double stiffness = 0;
        double damp = 0;
        double pos_limit[2]={0,0};
        double max_pos_err = 0.1745;
    }JointParam;

    /**
     * @brief axis state filter parameters
     */
    typedef struct StateFilterParam{
        /**
         * @brief state filter type enum
         */
        typedef enum FilterType{
            NONE = 0,
            LOW_PASS = 1,
            NOTCH = 2,
            KALMAN = 3,
            MIDDLE_VALUE = 4,
            AUTOKALMAN = 5
        } FilterType;

        FilterType type = FilterType::NONE;
        double low_pass_band = 0;   //unit: Hz /**< TODO: describe */
        double notch_center = 0;    //unit: Hz /**< TODO: describe */
        double kalman_Qvar = 0;
        double kalman_Rvar = 0;
        double middle_order = 0;
    }StateFilterParam;

    typedef struct{
        StateFilterParam pos_filter_param;
        StateFilterParam vel_filter_param;
        StateFilterParam acc_filter_param;
        StateFilterParam torq_filter_param;
        StateFilterParam auxi_pos_filter_param;
        StateFilterParam auxi_vel_filter_param;
        StateFilterParam auxi_acc_filter_param;
        StateFilterParam auxi_torq_filter_param;
    }AxisFilterParam;

    /**
     * @brief axis motion control parameters
     */
    typedef struct{
        /**
         * @brief axis motion control mode enum
         */
        enum class ControlType: uint16_t{
            NONE = 0,
            POSITION_PID = 1,
            VELOCITY_PI = 2,
            POSITION_PID_WITH_KA = 3,
            VELICITY_PI_WITH_KA = 4,
            PISITION_PPI = 5
        };
        ControlType ctrltype = ControlType::NONE; /**< specified axis motion control mode */
        double kp = 0; /**< proportional gain in specified control mode */
        double ki = 0; /**< integral gain in specified control mode */
        double kd = 0; /**< derivative gain in specified control mode */
        double ka = 0; /**< anti-windup gain in specified control mode */
    }CtrlParam;

    /**
     * @brief axis mode of operation enum
     */
    enum class MotionMode : int8_t {
        NONE = 0,
        PROFILE_POSITION = 1,
        PROFILE_VELOCITY = 3,
        PROFILE_TORQUE = 4,
        HOMING = 6,
        INTERPOLATED_POSITION = 7,
        CYCLIC_SYNCHRONOUS_POSITION = 8,
        CYCLIC_SYNCHRONOUS_VELOCITY = 9,
        CYCLIC_SYNCHRONOUS_TORQUE = 10
    };

    /**
     * @brief NC axis status enum
     */
    typedef enum NCAxisStatus{
        NONE = 0,
        DISABLE = 1,
        OPERATIONAL = 2,
        ERROR = 3
    } NCAxisStatus;

    /**
     * @brief axis motion state struct
     */
    typedef struct{
        double pos = 0;
        double vel = 0;
        double acc = 0;
        double torq = 0;
        MotionMode mode = MotionMode::NONE;
    }MotionState;

private:
    /**
     * @brief CIA402 fsa optional statusword flag
     */
    typedef struct{
        bool ReadyToSwitchOn = false;
        bool SwitchOn = false;
        bool OperationEnabled = false;
        bool Fault = false;
        bool VoltageEnabled = false;
        bool QuickStop = false;
        bool SwitchOnDisable = false;
        bool Warning = false;
        bool MS1 = false;
        bool Remote = false;
        bool TargetReached = false;
        bool InternalLimitActive = false;
        bool OMS[2] = {false,false};
        bool MS2[2] = {false,false};
    }FSAStateFlag;

    /**
     * @brief CIA402 FSA state machine enum
     */
    enum class FSAStateMachine : uint16_t {
        NOT_READY_TO_SWITCH_ON = 0,
        SWITCH_ON_DISABLED = 1,
        READY_TO_SWITCH_ON = 2,
        SWITCH_ON = 3,
        OPERATION_ENABLED = 4,
        QUICK_STOP_ACTIVE = 5,
        FAULT_REACTION_ACTIVE = 6,
        FAULT = 7
    };

    /**
     * @brief CIA402 FSA state machine enum
     */
    enum class FSAStateMachineMask : uint16_t {
        READY_TO_SWTICH_ON = 0x01,
        SWITCH_ON = 0x02,
        OPERATION_ENABLED = 0x04,
        FAULT = 0x08,
        VOLTAGE_ENABLED = 0x10,
        QUICK_STOP = 0x20,
        SWITCH_ON_DISABLE = 0x40,
        WARNING = 0x80,
        MS1 = 0x100,
        REMOTE = 0x200,
        TARGET_REACHED = 0x400,
        INTERNAL_LIMIT_ACTIVE = 0x800,
        OMS1 = 0x1000,
        OMS2 = 0x2000,
        MS2 = 0x4000,
        MS3 = 0x8000
    };

    /**
     * @brief CIA402 FSA state machine transition command enum
     */
    enum class FSAStateCommand : uint16_t {
        DISABLE_VOLTAGE = 0x0,
        QUIICK_STOP = 0x2,
        SHUT_DOWN = 0x6,
        SWITCH_ON = 0x7,
        DISABLE_OPERATION = 0x7,
        ENABLE_OPERATION = 0xf,
        FAULT_RESET = 0x80
    };

    /**
     * @brief CIA402 optional controlword
     */
    typedef struct{
        bool SwichOn = false;
        bool EnableVoltage=  false;
        bool QuickStop = false;
        bool EnableOperation = false;
        bool OMS1[3] = {false,false,false};
        bool FaultReset = false;
        bool Halt = false;
        bool OMS2 = false;
        bool Reserved = false;
        bool MS[5] = {false,false,false,false,false};
    }OptionalControlword;

public:
    /**
     * @brief NCAxis: constructor
     * @param axis_num: axis number index
     * @param axis_link: pointer to the corresponding physical axis
     */
    NCAxis(EcMaster* axis_link ,const double& cycle_time);
    /**
     * @brief destructor
     */
    ~NCAxis(){}

    /**
     * @brief inputUpdate: axis online input data update,
     *        including pdo data unit tranfer and protocal online resolving.
     *        make sure this function is called at the start of one executing cycle
     */
    virtual void inputUpdate();
    /**
     * @brief outputSend: axis online output data send,
     *        including pdo data unit transfer and protocal online resolving,
     *        make sure this function is called at the end of one executing cycle
     */
    virtual void outputSend();

    /**
      * @brief the following interfaces are CIA402 protocal related axis operation interfaces
      */
    /**
     * @brief enable: axis enable interface
     */
    bool enable();
    /**
     * @brief disable: axis disable interface
     */
    bool disable();
    /**
     * @brief reset: axis reset interface
     */
    bool reset();

    /**
      * @brief the following interfaces are physical hardware related parameters setter,
      *        the parameters must be set correctly based on the real parameters of corresponding hardwares
      */
    /**
     * @brief setEncoderParam: primary encoder perameters setter interface
     * @param primary_encoder_param: primary encoder parameters
     */
    void setPrimaryEncoderParam(const EncoderParam primary_encoder_param);
    /**
     * @brief setEncoderParam: auxiliary encoder perameters setter interface
     * @param auxiliary_encoder_param: auxiliary encoder parameters
     */
    void setAuxiliaryEncoderParam(const EncoderParam auxiliary_encoder_param);
    /**
     * @brief setMotorParam: motor parameters setter interface
     * @param motor_param: motor parameters
     */
    void setMotorParam(const MotorParam motor_param);
    /**
     * @brief setointParam: axis joint parameters setter interface
     * @param joint_param: axis parameters
     */
    void setJointParam(const JointParam joint_param);

    /**
     * @brief the following interfaces are motion control function related pamameters setter interface
     *        all parameters will effect axis motion behavior, thus must be set carefully
     */
    /**
     * @brief setFilterParam: axis motion state filter parameters setter interface
     * @param filter_param: axis motion state filter parameters
     */
    void setFilterParam(const AxisFilterParam& filter_param);
    /**
     * @brief setControlParam: axis motion control parameters setter interface
     * @param control_param: axis motion control parameters
     */
    void setControlParam(const CtrlParam& control_param);

    /**
     * @brief the following interfaces are axis online data interfaces
     */
    /**
     * @brief getAxisStatus: axis status getter interface
     * @return axis status online value
     */
    NCAxisStatus getAxisStatus();
    /**
     * @brief getPosition: axis position getter interface
     * @return axis position online value after position filter
     */
    virtual double getPosition();
    /**
     * @brief getVelocity: axis velocity getter interface
     * @return axis velocity online value after velocity filter
     */
    virtual double getVelocity();
    /**
     * @brief getAcceleration: axis acceleration getter interface
     * @return axis acceleration online value after acceleration filter
     */
    virtual double getAcceleration();
    /**
     * @brief getTorque: axis torque getter interface
     * @return axis torque online value after torque filter
     */
    virtual double getTorque();
    /**
     * @brief getCurrent
     * @return
     */
    virtual double getCurrent() const;
    /**
     * @brief getVoltage
     * @return
     */
    virtual double getVoltage() const;
    /**
     * @brief getTemperature
     * @return
     */
    virtual double getTemperature();
    /**
     * @brief getAuxiliaryPosition: axis auxiliary position getter interface
     * @return axis auxiliary position online value after auxiliary position filter
     */
    virtual double getAuxiliaryPosition();
    /**
     * @brief getAuxiliaryVelocity: axis auxiliary velocity getter interface
     * @return axis auxiliary velocity online value after auxiliary velocity filter
     */
    virtual double getAuxiliaryVelocity();
    /**
     * @brief getAuxiliaryAcceleration: axis auxiliary acceleration getter interface
     * @return axis auxiliary acceleration online value after auxiliary acceleration filter
     */
    virtual double getAuxiliaryAcceleration();
    /**
     * @brief getAuxiliaryTorque: axis auxiliary torque getter interface
     * @return axis auxiliary torque online value after auxilairy torque filter
     */
    double getAuxiliaryTorque();
    /**
     * @brief getMotionMode: axis mode of operation getter interface
     * @return axis mode of operation online value
     */
    MotionMode getMotionMode();
    /**
     * @brief setPosition: axis target position setter interface, only valid when axis operating in CSP mode
     * @param position: target axis position corresponding to primary encoder
     */
    virtual void setPosition(const double position);
    /**
     * @brief setVelocity: axis target velocity setter interface, only valid when axis operating in CSV mode
     * @param velocity: target axis velocity corresponding to auxiliary encoder
     */
    virtual void setVelocity(const double velocity);
    /**
     * @brief setAcceleration: axis target acceleration setter interface, only used for acceleration feedforward
     * @param acceleration: target axis acceleration corresponding to auxiliary encoder
     */
    virtual void setAcceleration(const double acceleration);
    /**
     * @brief setTorque: axis target torque setter interface, only valid when axis operating in CST mode
     * @param torque: target axis torque coresponding to motor
     */
    void setTorque(const double torque);
    /**
     * @brief setMotionMode: axis target mode of operation setter interface
     * @param motion_mode: target axis mode of operation
     */
    void setMotionMode(MotionMode motion_mode);
    /**
     * @brief switchSimulation: switch simulation mode of axis interface
     * @param bSimulation: switch on simulation when input true, switch off simulation when input false
     */
    void switchSimulation(bool& bSimulation);
    /**
     * @brief getErrId: axis error id getter interface
     * @return error id, return 0 when no error occurs or after reset
     */
    uint32_t getErrId();
    /**
     * @brief getErrMsg: axis error message getter interface
     * @param main_msg: main error message, return null when no error occurs or after reset
     * @param detailed_msg: detailed error message, return null when no error occurs or after reset
     */
    void getErrMsg(char* main_msg, char* detailed_msg);
    /**
     * @brief getAxis: axis object pointer getter interface
     * @return axis object pointer
     */
    //Axis402* getAxis(){return axis_link_;}
    /**
     * @brief switchPower
     * @param power_status
     */
    void switchPower(const bool& power_status);

    void setEcMaster(EcMaster *master);

    void setSlaveID(int id);

    void setAxisNum(int num);

    bool joint_invert=false;

    std::string joint_model;

    void setBrakeParam(const BrakeParam &param);

    int getSlaveID(){return this->slave_id;}


private:
    /**
     * @brief private class of transfer function for second order differential
     */
    class secondOrderDiff {
    public:
        /**
         * @brief constructor
         */
        explicit secondOrderDiff(double ts);

        /**
         * @brief deconstructor
         */
        ~secondOrderDiff(){}

        /**
         * @brief function execution
         * @param in: function input value
         * @param out1: function output after first order differential
         * @param out2: function output after second order differential
         */
        void excute(const double& in, double& out1, double& out2);

    private:
        double tmp1_,tmp2_; /**< local output variables */
        double ts_; /**< local execution cycle time */
    };

    /**
     * @brief the following interfaces are cia402 protocal related online update functions
     */
    /**
     * @brief fsaStateMachineUpdate: online resolve the cia402 fsa state machine transmission
     */
    void fsaStateMachineUpdate();
    /**
     * @brief fsaOptionalStateUpdate: online resolve the cia402 optional state in statusword
     */
    void fsaStateFlagUpdate();
    /**
     * @brief fsaTransitionMonitor: online monitor the cia402 fsa state machine transmission validation
     * @return transition error flag
     */
    bool fsaTransitionMonitor();
    /**
     * @brief fsaEventMonitor: online monitor the cia402 event occurance based on state machine and controlword validation
     * @return incorrect event occurance flag
     */
    bool fsaEventMonitor();
    /**
     * @brief axisStatusUpdate: online update axis output status based on cia402 fsa state machine
     */
    void axisStatusUpdate();
    /**
     * @brief setControlword: online resolve and send the cia402 controlword based on user operation command
     */
    void setControlword();
    /**
     * @brief stateFilter: filt the motion state signals based on user defined filter parameters
     */
    void stateFilter();
    /**
     * @brief controlStateCal: calculate the control variables of motion state based on user defined control parameters
     */
    void controlStateCal();



protected:
    EcMaster * axis_link_ = nullptr; /**< local pointer to physical axis */
    EcMaster::AxisPdoInputs axis_pdo_inputs_;
    EcMaster::AxisPdoOutputs axis_pdo_outputs_;
    uint32_t servo_errid_ = 0; /**< local servo error code */
    uint8_t nc_axis_errid_; /**< local axis errid */
    JointParam joint_param_;
    MotorParam motor_param_; /**< local motor parameters struct */
    char err_msg_[256]="\0"; /**< local axis error main message */
    MotionState origin_command_state_,output_command_state_; /**< local primary and auxiliary motion command states */
    MotionState origin_command_state_buffer_;
    double origin_command_jerk_buffer_;
    bool bsimulation_= false; /**< local simulation flag */
    EncoderParam primary_encoder_param_, auxiliary_encoder_param_; /**< local primary encoder parameters struct */
    CtrlParam ctrl_param_; /**< internal axis motion control parameters */
    BrakeParam brake_param;
    AxisFilterParam filter_param_; /**< internal axis motion state filter parameters */
    double ts_; /**< local cycle time */
    MotionState primary_origin_state_,auxiliary_origin_state_; /**< local primary and auxiliary motion origin states */
    MotionState primary_output_state_,auxiliary_output_state_; /**< local primary and auxiliary motion output states */
    double motor_temperature, reducer_temperature;

protected:
    bool power_status_=false;
    uint16_t statusword_ = 0; /**< local CIA402 statusword */
    uint16_t digital_outputs_ = 0; /**< local digital outputs */
    FSAStateFlag fsa_state_flag_; /**< local CIA402 optional statusword */
    FSAStateMachine fsa_state_machine_,previous_fsa_state_machine_; /**< local CIA402 state machine current and previous state */
    NCAxisStatus nc_axis_status_ = NCAxisStatus::NONE; /**< local axis state */
    uint16_t controlword_; /**< local CIA402 controlword */
    OptionalControlword optional_controlword_; /**< local CIA402 optional controlword */
    FSAStateCommand fsa_state_command_,previous_fsa_state_command_; /**< local CIA402 FSA events current and previous command */
    MotionState control_state_; /**< internal axis motion control states */
    secondOrderDiff* primary_diff_; /**< local differential tool for primary motion state */
    secondOrderDiff* auxiliary_diff_; /**< local differential tool for auxiliary motion state */
    int slave_id=0;
    int axis_num=0;
};
}

#endif
