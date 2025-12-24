/** @file　NCAxis.cpp
 *  @brief class of robot axis supports CIA402 protocol
 *  @author wangyuxing
 *  @date 2021
 *  @version 0.2.0
 *  @copyright Copyright(C)2019,SIASUN CO.,LTD.
 */
#include "NCAxis.h"
#include "string.h"
#include <iostream>

namespace DucoCobot {
NCAxis::NCAxis(EcMaster *axis_link, const double& cycle_time)
{
    /**
      * construct the property of class
      */
    axis_link_=axis_link;
    ts_ = cycle_time;
    bsimulation_ = false;
    nc_axis_errid_ = 0x00;
    memset(err_msg_,0,sizeof (err_msg_));
    memset(&axis_pdo_inputs_,0,sizeof (axis_pdo_inputs_));
    memset(&axis_pdo_outputs_,0,sizeof(axis_pdo_outputs_));
    memset(&primary_origin_state_,0,sizeof(primary_origin_state_));
    memset(&auxiliary_origin_state_,0,sizeof(auxiliary_origin_state_));
    memset(&primary_output_state_,0,sizeof(primary_output_state_));
    memset(&auxiliary_output_state_,0,sizeof(auxiliary_output_state_));
    memset(&origin_command_state_,0,sizeof(origin_command_state_));
    memset(&control_state_,0,sizeof(control_state_));
    /**
      * init the configuration parameters
      */
    primary_encoder_param_.invert = false;
    primary_encoder_param_.posbias = 0;
    primary_encoder_param_.scalefactor = 1;
    auxiliary_encoder_param_.invert = false;
    auxiliary_encoder_param_.posbias = 0;
    auxiliary_encoder_param_.scalefactor = 1;
    motor_param_.invert = false;
    motor_param_.kt = 1;
    motor_param_.ratedcurrent = 0;
    motor_param_.maxcurrent = 0;
    joint_param_.ratio = 101;
    joint_param_.rated_torque = 100;
    joint_param_.max_torque = 200;
    joint_param_.stiffness = 0;
    joint_param_.damp = 0;
    ctrl_param_.ctrltype=CtrlParam::ControlType::NONE;
    ctrl_param_.ka=0;
    ctrl_param_.kd=0;
    ctrl_param_.ki=0;
    ctrl_param_.kp=0;
    memset(&filter_param_,0,sizeof (filter_param_));
    /**
      * init second order differential object
      */
    primary_diff_ = new secondOrderDiff(ts_);
    auxiliary_diff_ = new secondOrderDiff(ts_);
    /**
      * init local cia402 FSA variables
      */
    fsa_state_machine_ = FSAStateMachine::NOT_READY_TO_SWITCH_ON;
    previous_fsa_state_machine_ = FSAStateMachine::NOT_READY_TO_SWITCH_ON;
    fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
    previous_fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
    /**
      * init the local motion state variables
      */
    control_state_.torq = 0;
    control_state_.acc = 0;
    control_state_.vel = 0;
    control_state_.pos = 0;

    /**
      * init motor and reducer temperature with 25
      */
    motor_temperature = MIN_REDUCER_TEMPERATURE;
    reducer_temperature = MIN_REDUCER_TEMPERATURE;
}

void NCAxis::inputUpdate()
{
    //CIA402 handler
    if (axis_link_ && axis_link_->notSimulation())
    {
        //        EC_T_STATE ec_state = EC_T_STATE::eEcatState_INIT;
        //        axis_link_->getEcState(ec_state);
        //        if (EC_T_STATE::eEcatState_OP == ec_state)
        axis_pdo_inputs_ = axis_link_->getAxisPdoInputs(this->axis_num);
        //        else
        //            memset(&axis_pdo_inputs_,0,sizeof(axis_pdo_inputs_));

        /*Servo error update*/
        servo_errid_ = axis_pdo_inputs_.ErrorCode;
        for (unsigned int j=0;j<8*sizeof(uint32_t);++j)
        {
            if ((servo_errid_>>j)&1)
            {
                nc_axis_errid_=static_cast<uint8_t>(j+1);
            }
        }
        /*CIA402*/
        statusword_ = axis_pdo_inputs_.Statusword;
        fsaStateFlagUpdate();
        fsaStateMachineUpdate();
        axisStatusUpdate();
    }
    else
    {
        if (!power_status_)
            nc_axis_status_ = NCAxisStatus::NONE;
        else if (nc_axis_status_ == NCAxisStatus::NONE)
            nc_axis_status_ = NCAxisStatus::DISABLE;
    }

    /*Axis state update*/
    if (!bsimulation_ && axis_link_ && axis_link_->notSimulation())
    {
        if (nc_axis_status_ == NCAxisStatus::NONE){
            primary_origin_state_.acc=0;
            primary_origin_state_.vel=0;
            primary_origin_state_.pos=0;
            auxiliary_origin_state_.acc=0;
            auxiliary_origin_state_.vel=0;
            auxiliary_origin_state_.pos=0;
        }
        else
        {
            int32_t PrimaryPosCnt = 0,PrimaryVelCnt = 0;
            double PrimaryJerkTmp = 0;
            double primary_encoder_dir_;
            //primary motion state calculation
            PrimaryPosCnt = axis_pdo_inputs_.ActualPosition;
            primary_encoder_dir_ = primary_encoder_param_.invert ? -1 : 1;
            primary_origin_state_.pos = PrimaryPosCnt*primary_encoder_param_.scalefactor*primary_encoder_dir_-primary_encoder_param_.posbias;
            PrimaryVelCnt = axis_pdo_inputs_.ActualVelocity;
            primary_origin_state_.vel = PrimaryVelCnt*primary_encoder_param_.scalefactor*primary_encoder_dir_;
            primary_diff_->excute(primary_origin_state_.vel,primary_origin_state_.acc,PrimaryJerkTmp);
            //auxiliary motion state calculation
            int32_t AuxiliaryPosCnt=0,AuxiliaryVelCnt=0;
            double AuxiliaryJerkTmp=0;
            double  auxiliary_encoder_dir_;
            AuxiliaryPosCnt = axis_pdo_inputs_.AuxiliaryPosition;
            auxiliary_encoder_dir_ = auxiliary_encoder_param_.invert ? -1 : 1;
            auxiliary_origin_state_.pos = double(AuxiliaryPosCnt)*primary_encoder_param_.scalefactor*auxiliary_encoder_dir_-primary_encoder_param_.posbias;
            AuxiliaryVelCnt = axis_pdo_inputs_.AuxiliaryVelocity;
            auxiliary_origin_state_.vel = double(AuxiliaryVelCnt)*primary_encoder_param_.scalefactor*auxiliary_encoder_dir_;
            auxiliary_diff_->excute(auxiliary_origin_state_.vel,auxiliary_origin_state_.acc,AuxiliaryJerkTmp);
        }
    }
    else
    {
        primary_origin_state_.pos=output_command_state_.pos;
        primary_origin_state_.vel=output_command_state_.vel;
        primary_origin_state_.acc=output_command_state_.acc;
        auxiliary_origin_state_.pos=output_command_state_.pos;
        auxiliary_origin_state_.vel=output_command_state_.vel;
        auxiliary_origin_state_.acc=output_command_state_.acc;
    }

    if (axis_link_ && axis_link_->notSimulation())
    {
        double motor_dir_ = 0;
        double fric_torq_ = 0, inertia_torq_ = 0;
        //motion mode update
        primary_origin_state_.mode = MotionMode(axis_pdo_inputs_.ModeOfOperationDisplay);
        //current\torque update
        int16_t current_original_val=0,torque_original_val=0;
        int16_t current_auxiliary_val=0,torque_auxiliary_val=0;
        //currently servo return current signal as torque value, need to be update after joint torque sensor is integrated
        //torque_original_val = axis_pdo_inputs_.ActualTorque;
        //torque_auxiliary_val = axis_pdo_inputs_.AuxiliaryTorque;
        current_original_val = axis_pdo_inputs_.ActualTorque;
        current_auxiliary_val = axis_pdo_inputs_.AuxiliaryTorque;
        motor_dir_ =  motor_param_.invert ? -1 : 1;
        fric_torq_ =  0;
        inertia_torq_ = motor_param_.inertia*auxiliary_output_state_.acc;
        primary_origin_state_.torq = (double(current_original_val)*motor_dir_-fric_torq_)/1000*motor_param_.ratedcurrent*motor_param_.kt*joint_param_.ratio-inertia_torq_;
        auxiliary_origin_state_.torq = (double(current_auxiliary_val)*motor_dir_-fric_torq_)/1000*motor_param_.ratedcurrent*motor_param_.kt*joint_param_.ratio-inertia_torq_;
    }
    else
    {
        if (nc_axis_status_ == NCAxisStatus::OPERATIONAL)
        {
            primary_origin_state_.mode = output_command_state_.mode;
            primary_origin_state_.torq = output_command_state_.torq;
        }
        else
        {
            primary_origin_state_.mode = MotionMode::NONE;
            primary_origin_state_.torq = 0;
        }
    }

    /* output motion state variables calculation, including pos/vel/acc filtering and mode transfer */
    stateFilter();

    primary_output_state_.mode = primary_origin_state_.mode;
}

void NCAxis::outputSend()
{
    /* control state variables calculation and superposition */
    controlStateCal();
    output_command_state_.pos = origin_command_state_.pos + control_state_.pos;
    output_command_state_.vel = origin_command_state_.vel + control_state_.vel;
    output_command_state_.acc = origin_command_state_.acc + control_state_.acc;
    output_command_state_.torq = origin_command_state_.torq + control_state_.torq;
    output_command_state_.mode = origin_command_state_.mode;

    /* update local pdo outputs variables */
    double command_torque=0,command_current=0;
    if (axis_link_ && axis_link_->notSimulation())
    {
        setControlword();
        axis_pdo_outputs_.ModeOfOperation = int8_t(output_command_state_.mode);
        axis_pdo_outputs_.Controlword = controlword_;
        axis_pdo_outputs_.DigitalOutputs = digital_outputs_;

        double motor_dir_ = 0;
        double fric_torq_ = 0, inertia_torq_ = 0;
        motor_dir_ = motor_param_.invert ? -1 : 1;
        fric_torq_ =  0;
        inertia_torq_ = motor_param_.inertia*output_command_state_.acc;
        command_torque = output_command_state_.torq;
        if (command_torque < -joint_param_.max_torque)
            command_torque = -joint_param_.max_torque;
        else if (command_torque > -joint_param_.max_torque)
            command_torque = joint_param_.max_torque;
        command_current = (output_command_state_.torq+inertia_torq_)/joint_param_.ratio/motor_param_.kt/motor_param_.ratedcurrent*1000+fric_torq_;
        if (command_current < -motor_param_.maxcurrent/motor_param_.ratedcurrent*1000)
            command_current = -motor_param_.maxcurrent/motor_param_.ratedcurrent*1000;
        else if (command_current > motor_param_.maxcurrent/motor_param_.ratedcurrent*1000)
            command_current = motor_param_.maxcurrent/motor_param_.ratedcurrent*1000;
    }

    /* update local motion command pdo outputs variables when simulation mode is off */
    if (axis_link_ && !bsimulation_)
    {
        /*Axis state unit transfer*/
        double primary_encoder_dir_;
        primary_encoder_dir_ = primary_encoder_param_.invert ? -1 : 1;
        axis_pdo_outputs_.TargetPosition = int32_t((output_command_state_.pos+primary_encoder_param_.posbias)*primary_encoder_dir_/primary_encoder_param_.scalefactor);
        axis_pdo_outputs_.TargetVelocity = int32_t(output_command_state_.vel*primary_encoder_dir_/primary_encoder_param_.scalefactor);
        //currently servo receive current signal command as torque command, need to be update after joint torque sensor is integrated
        //        axis_pdo_outputs_.TargetTorque = int16_t(command_torque);
        axis_pdo_outputs_.TargetTorque = int16_t(command_current);
    }

    if (axis_link_ && axis_link_->notSimulation())
    {
        axis_link_->setAxisPdoOutputs(axis_pdo_outputs_,this->axis_num);
    }
}

bool NCAxis::enable()
{
    bool ret = false;
    if (axis_link_ && axis_link_->notSimulation())
    {
        if (fsa_state_machine_ == FSAStateMachine::NOT_READY_TO_SWITCH_ON)
        {
            fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED)
        {
            fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
        }
        else if (fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON)
        {
            fsa_state_command_ = FSAStateCommand::SWITCH_ON;
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON)
        {
            fsa_state_command_ = FSAStateCommand::ENABLE_OPERATION;
        }
        else if (fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED)
        {
            ret = true;
        }
        else if (fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE)
        {
            fsa_state_command_ = FSAStateCommand::ENABLE_OPERATION;
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE)
        {
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT)
        {
        }
        else
        {
        }
    }
    else
    {
        nc_axis_status_ = NCAxisStatus::OPERATIONAL;
        ret = true;
    }
    return ret;
}

bool NCAxis::disable()
{
    bool ret=false;
    if (axis_link_ && axis_link_->notSimulation()){
        if (fsa_state_machine_ == FSAStateMachine::NOT_READY_TO_SWITCH_ON)
        {
            ret = true;
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED)
        {
            ret = true;
        }
        else if (fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON)
        {
            ret = true;
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON)
        {
            fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
        }
        else if (fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED)
        {
            fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
        }
        else if (fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE)
        {
            fsa_state_command_ = FSAStateCommand::DISABLE_VOLTAGE;
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE)
        {
            ret = true;
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT)
        {
            ret = true;
        }
        else
        {
        }
    }
    else
    {
        if (NCAxisStatus::OPERATIONAL == nc_axis_status_)
            nc_axis_status_=NCAxisStatus::DISABLE;
        ret = true;
    }
    return ret;
}

bool NCAxis::reset()
{
    if (axis_link_ && axis_link_->notSimulation())
    {
        if (fsa_state_machine_ == FSAStateMachine::FAULT)
        {
            fsa_state_command_ = FSAStateCommand::FAULT_RESET;
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE)
        {
            fsa_state_command_ = FSAStateCommand::FAULT_RESET;
        }
        else if (fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE)
        {
            fsa_state_command_ = FSAStateCommand::DISABLE_VOLTAGE;
        }
        else
        {
            fsa_state_command_ = FSAStateCommand::SHUT_DOWN;
        }
    }
    if (NCAxisStatus::NONE != nc_axis_status_)
        nc_axis_status_ = NCAxisStatus::DISABLE;
    previous_fsa_state_command_ = fsa_state_command_;
    previous_fsa_state_machine_ = fsa_state_machine_;
    nc_axis_errid_=0x00;
    strcpy(err_msg_,"");
    return true;
}

void NCAxis::setPrimaryEncoderParam(const EncoderParam primary_encoder_param)
{
    primary_encoder_param_ = primary_encoder_param;
}

void NCAxis::setAuxiliaryEncoderParam(const EncoderParam auxiliary_encoder_param)
{
    auxiliary_encoder_param_ = auxiliary_encoder_param;
}

void NCAxis::setMotorParam(const MotorParam motor_param)
{
    motor_param_ = motor_param;
}

void NCAxis::setJointParam(const JointParam joint_param)
{
    joint_param_ = joint_param;
}

void NCAxis::setFilterParam(const AxisFilterParam &filter_param)
{
    filter_param_ = filter_param;
}

void NCAxis::setControlParam(const CtrlParam &control_param)
{
    ctrl_param_ = control_param;
}

NCAxis::NCAxisStatus NCAxis::getAxisStatus()
{
    return nc_axis_status_;
}

double NCAxis::getPosition()
{
    return primary_output_state_.pos;
}

double NCAxis::getVelocity()
{
    return primary_output_state_.vel;
}

double NCAxis::getAcceleration()
{
    return primary_output_state_.acc;
}

double NCAxis::getTorque()
{
    return primary_output_state_.torq;
}

double NCAxis::getCurrent() const
{
    return 0;
}

double NCAxis::getVoltage() const
{
    return 0;
}

double NCAxis::getTemperature()
{
    motor_temperature = 0;
    reducer_temperature = 0;
    return reducer_temperature;
}

double NCAxis::getAuxiliaryPosition()
{
    return auxiliary_output_state_.pos;
}

double NCAxis::getAuxiliaryVelocity()
{
    return auxiliary_output_state_.vel;
}

double NCAxis::getAuxiliaryAcceleration()
{
    return auxiliary_output_state_.acc;
}

double NCAxis::getAuxiliaryTorque()
{
    return auxiliary_output_state_.torq;
}

NCAxis::MotionMode NCAxis::getMotionMode()
{
    return primary_output_state_.mode;
}

void NCAxis::setPosition(const double position)
{
    origin_command_state_.pos = position;
}

void NCAxis::setVelocity(const double velocity)
{
    origin_command_state_.vel = velocity;
}

void NCAxis::setAcceleration(const double acceleration)
{
    origin_command_state_.acc = acceleration;
}

void NCAxis::setTorque(const double torque)
{
    origin_command_state_.torq = torque;
}

void NCAxis::setMotionMode(const MotionMode motion_mode)
{
    origin_command_state_.mode = motion_mode;
}

void NCAxis::switchSimulation(bool &bSimulation)
{
    bsimulation_ = bSimulation;
}

uint32_t NCAxis::getErrId()
{
    return (nc_axis_errid_ == 0) ? 0 : 0x13030000+static_cast<uint32_t>(nc_axis_errid_);
}

void NCAxis::getErrMsg(char *main_msg, char *detailed_msg)
{

}

void NCAxis::switchPower(const bool &power_status)
{
    power_status_=power_status;
}


void NCAxis::fsaStateMachineUpdate()
{
    if((fsa_state_flag_.ReadyToSwitchOn == false) & (fsa_state_flag_.SwitchOn == false) &
            (fsa_state_flag_.OperationEnabled == false) & (fsa_state_flag_.Fault == false) &
            (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::NOT_READY_TO_SWITCH_ON;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == false) & (fsa_state_flag_.SwitchOn == false) &
            (fsa_state_flag_.OperationEnabled == false) & (fsa_state_flag_.Fault == false) &
            (fsa_state_flag_.SwitchOnDisable == true))
    {
        fsa_state_machine_ = FSAStateMachine::SWITCH_ON_DISABLED;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == true) & (fsa_state_flag_.SwitchOn == false) &
            (fsa_state_flag_.OperationEnabled == false) & (fsa_state_flag_.Fault == false) &
            (fsa_state_flag_.QuickStop == true) & (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::READY_TO_SWITCH_ON;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == true) & (fsa_state_flag_.SwitchOn == true) &
            (fsa_state_flag_.OperationEnabled == false) & (fsa_state_flag_.Fault == false) &
            (fsa_state_flag_.QuickStop == true) & (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::SWITCH_ON;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == true) & (fsa_state_flag_.SwitchOn == true) &
            (fsa_state_flag_.OperationEnabled == true) & (fsa_state_flag_.Fault == false) &
            (fsa_state_flag_.QuickStop == true) & (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::OPERATION_ENABLED;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == true) & (fsa_state_flag_.SwitchOn == true) &
            (fsa_state_flag_.OperationEnabled == true) & (fsa_state_flag_.Fault == false) &
            (fsa_state_flag_.QuickStop == false) & (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::QUICK_STOP_ACTIVE;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == true) & (fsa_state_flag_.SwitchOn == true) &
            (fsa_state_flag_.OperationEnabled == true) & (fsa_state_flag_.Fault == true) &
            (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::FAULT_REACTION_ACTIVE;
    }
    else if((fsa_state_flag_.ReadyToSwitchOn == false) & (fsa_state_flag_.SwitchOn == false) &
            (fsa_state_flag_.OperationEnabled == false) & (fsa_state_flag_.Fault == true) &
            (fsa_state_flag_.SwitchOnDisable == false))
    {
        fsa_state_machine_ = FSAStateMachine::FAULT;
    }
}

void NCAxis::fsaStateFlagUpdate()
{
    fsa_state_flag_.ReadyToSwitchOn = ((statusword_ & uint16_t(FSAStateMachineMask::READY_TO_SWTICH_ON)) == uint16_t(FSAStateMachineMask::READY_TO_SWTICH_ON));
    fsa_state_flag_.SwitchOn = ((statusword_ & uint16_t(FSAStateMachineMask::SWITCH_ON)) == uint16_t(NCAxis::FSAStateMachineMask::SWITCH_ON));
    fsa_state_flag_.OperationEnabled = ((statusword_ & uint16_t(NCAxis::FSAStateMachineMask::OPERATION_ENABLED)) == uint16_t(FSAStateMachineMask::OPERATION_ENABLED));
    fsa_state_flag_.Fault = ((statusword_ & uint16_t(FSAStateMachineMask::FAULT)) == uint16_t(FSAStateMachineMask::FAULT));
    fsa_state_flag_.VoltageEnabled = ((statusword_ & uint16_t(FSAStateMachineMask::VOLTAGE_ENABLED)) == uint16_t(FSAStateMachineMask::VOLTAGE_ENABLED));
    fsa_state_flag_.QuickStop = ((statusword_ & uint16_t(FSAStateMachineMask::QUICK_STOP)) == uint16_t(FSAStateMachineMask::QUICK_STOP));
    fsa_state_flag_.SwitchOnDisable = ((statusword_ & uint16_t(NCAxis::FSAStateMachineMask::SWITCH_ON_DISABLE)) == uint16_t(NCAxis::FSAStateMachineMask::SWITCH_ON_DISABLE));
    fsa_state_flag_.Warning = ((statusword_ & uint16_t(FSAStateMachineMask::WARNING)) == uint16_t(FSAStateMachineMask::WARNING));
    fsa_state_flag_.MS1 = ((statusword_ & uint16_t(NCAxis::FSAStateMachineMask::MS1)) == uint16_t(NCAxis::FSAStateMachineMask::MS1));
    fsa_state_flag_.Remote = ((statusword_ & uint16_t(FSAStateMachineMask::REMOTE)) == uint16_t(FSAStateMachineMask::REMOTE));
    fsa_state_flag_.TargetReached = ((statusword_ & uint16_t(FSAStateMachineMask::TARGET_REACHED)) == uint16_t(FSAStateMachineMask::TARGET_REACHED));
    fsa_state_flag_.InternalLimitActive = ((statusword_ & uint16_t(NCAxis::FSAStateMachineMask::INTERNAL_LIMIT_ACTIVE)) == uint16_t(NCAxis::FSAStateMachineMask::INTERNAL_LIMIT_ACTIVE));
    fsa_state_flag_.OMS[0] = ((statusword_ & uint16_t(FSAStateMachineMask::OMS1)) == uint16_t(FSAStateMachineMask::OMS1));
    fsa_state_flag_.OMS[1] = ((statusword_ & uint16_t(FSAStateMachineMask::OMS2)) == uint16_t(FSAStateMachineMask::OMS2));
    fsa_state_flag_.MS2[0] = ((statusword_ & uint16_t(FSAStateMachineMask::MS2)) == uint16_t(FSAStateMachineMask::MS2));
    fsa_state_flag_.MS2[1] = ((statusword_ & uint16_t(FSAStateMachineMask::MS3)) == uint16_t(FSAStateMachineMask::MS3));
}

bool NCAxis::fsaTransitionMonitor()
{
    bool err = false;
    if (fsa_state_machine_ != previous_fsa_state_machine_)
    {
        err = true;
        if (!power_status_ && fsa_state_machine_ == FSAStateMachine::NOT_READY_TO_SWITCH_ON)
        {
            err = false;
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::NOT_READY_TO_SWITCH_ON)
        {
            if ((fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT))
            {                   /*transition 1*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED)
        {
            if ((fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE))
            {                   /*transition 2*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON)
        {
            if ((fsa_state_machine_ == FSAStateMachine::SWITCH_ON) |                          /*transition 3*/
                    (fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE))
            {                 /*transition 7*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::SWITCH_ON)
        {
            if ((fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED) |                 /*transition 10*/
                    (fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED) |                  /*transition 4*/
                    (fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE))
            {                 /*transition 6*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED)
        {
            if ((fsa_state_machine_ == FSAStateMachine::SWITCH_ON) |                          /*transition 5*/
                    (fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON) |                 /*transition 8*/
                    (fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED) |                 /*transition 9*/
                    (fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE))
            {                  /*transition 11*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE)
        {
            if ((fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED) |                 /*transition 12*/
                    (fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED) |
                    (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE))
            {                  /*transition 16*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::FAULT)
        {
            if ((fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED) |
                    (fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON))
            {                   /*transition 15*/
                err = false;
            }
        }
        else if (previous_fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE)
        {
            if (fsa_state_machine_ == FSAStateMachine::FAULT)
            {                                /*transition 14*/
                err = false;
            }
        }
        else
        {
            err = true;
            nc_axis_errid_ = 0x60;
        }
    }
    else
    {
        err = false;
    }
    if (!err)
    {
        previous_fsa_state_machine_ = fsa_state_machine_;
    }
    return err;
}

bool NCAxis::fsaEventMonitor()
{
    bool err = false;
    if (fsa_state_command_ != previous_fsa_state_command_)
    {
        err = true;
        if (fsa_state_machine_ == FSAStateMachine::NOT_READY_TO_SWITCH_ON)
        {               /*event 1*/
            err = false;
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED)
        {
            if (fsa_state_command_ == FSAStateCommand::SHUT_DOWN)
            {                /*event 2*/
                err = false;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON)
        {
            if ((fsa_state_command_ == FSAStateCommand::SWITCH_ON) |              /*event 3*/
                    (fsa_state_command_ == FSAStateCommand::QUIICK_STOP) |            /*event 7*/
                    (fsa_state_command_ == FSAStateCommand::DISABLE_VOLTAGE))
            {        /*event 7*/
                err = false;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON)
        {
            if ((fsa_state_command_ == FSAStateCommand::ENABLE_OPERATION) |              /*event 4*/
                    (fsa_state_command_ == FSAStateCommand::SHUT_DOWN) |                 /*event 6*/
                    (fsa_state_command_ == FSAStateCommand::DISABLE_VOLTAGE))
            {        /*event 10*/
                err = false;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED)
        {              /*event 5*/
            if ((fsa_state_command_ == FSAStateCommand::DISABLE_OPERATION) |             /*event 8*/
                    (fsa_state_command_ == FSAStateCommand::SHUT_DOWN) |                 /*event 9*/
                    (fsa_state_command_ == FSAStateCommand::DISABLE_VOLTAGE) |           /*event 11*/
                    (fsa_state_command_ == FSAStateCommand::QUIICK_STOP))
            {
                err = false;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE)
        {
            if ((fsa_state_command_ == FSAStateCommand::ENABLE_OPERATION) |              /*event 16*/
                    (fsa_state_command_ == FSAStateCommand::DISABLE_VOLTAGE))            /*event 12*/
            {
                err = false;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE)          /*event 14*/
        {
            err = false;
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT)
        {
            if (fsa_state_command_ == FSAStateCommand::FAULT_RESET)                     /*event 15*/
            {
                err = false;
            }
        }
        else
        {
            err= true;
            nc_axis_errid_ = 0x61;
        }
    }
    else
    {
        err = false;
    }
    if (!err)
    {
        previous_fsa_state_command_ = fsa_state_command_;
    }
    return err;
}

void NCAxis::axisStatusUpdate()
{
    bool err = fsaTransitionMonitor();
    if (err)
    {
        nc_axis_status_ = NCAxisStatus::ERROR;
    }
    else
    {
        if (fsa_state_machine_ == FSAStateMachine::NOT_READY_TO_SWITCH_ON)
        {
            nc_axis_status_ = NCAxisStatus::NONE;
        }
        else if(fsa_state_machine_ == FSAStateMachine::SWITCH_ON_DISABLED)
        {
            nc_axis_status_ = NCAxisStatus::DISABLE;
        }
        else if(fsa_state_machine_ == FSAStateMachine::READY_TO_SWITCH_ON)
        {
            nc_axis_status_ = NCAxisStatus::DISABLE;
        }
        else if (fsa_state_machine_ == FSAStateMachine::SWITCH_ON)
        {
            nc_axis_status_ = NCAxisStatus::DISABLE;
        }
        else if (fsa_state_machine_ == FSAStateMachine::OPERATION_ENABLED)
        {
            if (fsa_state_flag_.VoltageEnabled)
            {
                nc_axis_status_ = NCAxisStatus::OPERATIONAL;
            }
            else
            {
                nc_axis_status_ = NCAxisStatus::DISABLE;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::QUICK_STOP_ACTIVE)
        {
            if (fsa_state_flag_.VoltageEnabled)
            {
                nc_axis_status_ = NCAxisStatus::OPERATIONAL;
            }
            else
            {
                nc_axis_status_ = NCAxisStatus::DISABLE;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT_REACTION_ACTIVE)
        {
            if (servo_errid_ == 0 || !power_status_)
            {
                nc_axis_status_ = NCAxisStatus::NONE;
            }
            else
            {
                nc_axis_status_ = NCAxisStatus::ERROR;
            }
        }
        else if (fsa_state_machine_ == FSAStateMachine::FAULT)
        {
            if (servo_errid_ == 0 || !power_status_)
            {
                nc_axis_status_ = NCAxisStatus::NONE;
            }
            else
            {
                nc_axis_status_ = NCAxisStatus::ERROR;
            }
        }
        else
        {
            nc_axis_status_ = NCAxisStatus::ERROR;
        }
    }
}

void NCAxis::setControlword()
{
    FSAStateCommand command;
    bool err = fsaEventMonitor();
    if (err)
    {
        nc_axis_status_ = NCAxisStatus::ERROR;
        command = previous_fsa_state_command_;
    }
    else
    {
        command = fsa_state_command_;
    }
    if (command == FSAStateCommand::SHUT_DOWN)
    {
        optional_controlword_.SwichOn = false;
        optional_controlword_.EnableVoltage = true;
        optional_controlword_.QuickStop = true;
        optional_controlword_.FaultReset = false;
    }
    else if (command == FSAStateCommand::SWITCH_ON)
    {
        optional_controlword_.SwichOn = true;
        optional_controlword_.EnableVoltage = true;
        optional_controlword_.QuickStop = true;
        optional_controlword_.EnableOperation = false;
        optional_controlword_.FaultReset = false;
    }
    else if (command == FSAStateCommand::ENABLE_OPERATION)
    {
        optional_controlword_.SwichOn = true;
        optional_controlword_.EnableVoltage = true;
        optional_controlword_.QuickStop = true;
        optional_controlword_.EnableOperation = true;
        optional_controlword_.FaultReset = false;
    }
    else if (command == FSAStateCommand::DISABLE_VOLTAGE)
    {
        optional_controlword_.EnableVoltage = false;
        optional_controlword_.FaultReset = false;
    }
    else if (command == FSAStateCommand::QUIICK_STOP)
    {
        optional_controlword_.EnableVoltage = true;
        optional_controlword_.QuickStop = false;
        optional_controlword_.FaultReset = false;
    }
    else if (command == FSAStateCommand::DISABLE_OPERATION)
    {
        optional_controlword_.SwichOn = true;
        optional_controlword_.EnableVoltage = true;
        optional_controlword_.QuickStop = true;
        optional_controlword_.EnableOperation = false;
        optional_controlword_.FaultReset = false;
    }
    else if (command == FSAStateCommand::FAULT_RESET)
    {
        optional_controlword_.FaultReset = true;
    }
    else{
        optional_controlword_.SwichOn = false;
        optional_controlword_.EnableVoltage = true;
        optional_controlword_.QuickStop = true;
        optional_controlword_.FaultReset = false;
    }
    controlword_ = uint16_t(optional_controlword_.SwichOn) + (uint16_t(optional_controlword_.EnableVoltage) << 1) +
            (uint16_t(optional_controlword_.QuickStop) << 2) + (uint16_t(optional_controlword_.EnableOperation) << 3) +
            (uint16_t(optional_controlword_.OMS1[0]) << 4) + (uint16_t(optional_controlword_.OMS1[1]) << 5) +
            (uint16_t(optional_controlword_.OMS1[2]) << 6) + (uint16_t(optional_controlword_.FaultReset) << 7) +
            (uint16_t(optional_controlword_.Halt) << 8) + (uint16_t(optional_controlword_.OMS2) << 9) +
            (uint16_t(optional_controlword_.Reserved) << 10) + (uint16_t(optional_controlword_.MS[0]) << 11) +
            (uint16_t(optional_controlword_.MS[1]) << 12) + (uint16_t(optional_controlword_.MS[2]) << 13) +
            (uint16_t(optional_controlword_.MS[3]) << 14) + (uint16_t(optional_controlword_.MS[4]) << 15);
}

void NCAxis::stateFilter()
{
    //axis position filter
    primary_output_state_.pos = primary_origin_state_.pos;
    //axis velocity filter
    primary_output_state_.vel = primary_origin_state_.vel;
    //axis acceleration filter
    primary_output_state_.acc = primary_origin_state_.acc;
    //axis torque filter
    primary_output_state_.torq = primary_origin_state_.torq;
    //auxiliary axis position filter
    auxiliary_output_state_.pos = auxiliary_origin_state_.pos;
    //auxiliary axis velocity filter
    auxiliary_output_state_.vel = auxiliary_origin_state_.vel;
    //auxiliary axis acceleration filter
    auxiliary_output_state_.acc = auxiliary_origin_state_.acc;
    //auxiliary axis torque filter
    auxiliary_output_state_.torq = auxiliary_origin_state_.torq;
}

void NCAxis::controlStateCal()
{
    /*Reserved for control algorithm integration*/
    if (ctrl_param_.ctrltype == CtrlParam::ControlType::NONE)
    {
    }
    else if (ctrl_param_.ctrltype == CtrlParam::ControlType::VELOCITY_PI)
    {
        control_state_.torq = 0;
    }
    else if (ctrl_param_.ctrltype == CtrlParam::ControlType::POSITION_PID)
    {
        control_state_.vel = 0;
        control_state_.torq = 0;
    }
    else if (ctrl_param_.ctrltype == CtrlParam::ControlType::VELICITY_PI_WITH_KA)
    {
        control_state_.torq = 0;
    }
    else if (ctrl_param_.ctrltype == CtrlParam::ControlType::POSITION_PID_WITH_KA)
    {
        control_state_.vel = 0;
        control_state_.torq = 0;
    }
    else
    {

    }
}

void NCAxis::setEcMaster(EcMaster *master)
{
    this->axis_link_=master;
}

void NCAxis::setSlaveID(int id)
{
    this->slave_id=id;
}

void NCAxis::setAxisNum(int num)
{
    this->axis_num=num;
}

void NCAxis::setBrakeParam(const NCAxis::BrakeParam &param)
{
    this->brake_param=param;
}

NCAxis::secondOrderDiff::secondOrderDiff(double ts)
{
    ts_ = ts;
    tmp1_ = 0;
    tmp2_ = 0;
}

void NCAxis::secondOrderDiff::excute(const double &in, double &out1, double &out2)
{
    if (ts_ == 0)
    {
        out1 = 0;
        out2 = 0;
        return;
    }
    out1 = (in-tmp1_)/ts_;
    out2 = (in-2*tmp1_+tmp2_)/pow(ts_,2);
    tmp2_ = tmp1_;
    tmp1_ = in;
}
}

