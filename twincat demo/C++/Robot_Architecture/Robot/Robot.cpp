#include "Robot.h"
#include "NCAxis.h"
#include "EcMaster/EcMaster.h"
#include <string>
#include <iostream>
#include <fstream>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include "Control/Control.h"
#include "Robot/AxisControl.h"
#include "Robot/AxisSafeControl.h"
#include "Robot/TcpControl.h"
namespace DucoCobot {
Robot::Robot(EcMaster* master)
{
    this->master=master;
    init();
}

int Robot::init()
{
    for(int i=0;i<AXIS_NUM;i++)
    {
        AxisSafeControl *axis=new AxisSafeControl(master,ARM_CYCLE_TIME);

        //The slave ID number needs to be configured according to the user's master station.
        axis->setSlaveID(i);
        axis->setAxisNum(i);
        axises.push_back(axis);
    }
    tcp_board=new TcpControl(master);

    //The slave ID number needs to be configured according to the user's master station.
    tcp_board->setSlaveID(AXIS_NUM);
    return 0;
}

int Robot::uploadFileFromTcpBoard()
{
    if(!master)
        return -1;
    std::string root_path="./";
    char* c_name = "robot_sn";
    char data[FOEMAX];
    memset(data,0,FOEMAX);
    int recv_data_len;
    int ret=0;
    ret=master->foeFileUpload(tcp_board->getSlaveID(),c_name,strlen(c_name),data,FOEMAX,&recv_data_len,SDOTIMEOUT);
    if(ret!=0)
    {
        return ret;
    }
    std::ofstream ofile;
    std::string file_save=Global::root_file+"/robot.json";
    ofile.open(file_save.data());
    ofile.write(data,recv_data_len);
    ofile.flush();
    ofile.close();


    char *c_name2="robot_param";
    int recv_data_len2;
    memset(data,0,FOEMAX);
    ret=master->foeFileUpload(tcp_board->getSlaveID(),c_name2,strlen(c_name2),data,FOEMAX,&recv_data_len2,SDOTIMEOUT);
    if(ret!=0)
        return ret;
    file_save=Global::root_file+"/config.tar.gz";
    ofile.open(file_save.data());
    ofile.write(data,recv_data_len2);
    ofile.flush();
    ofile.close();
    std::string config_path=Global::root_file+"/config";
    std::string command=(std::string)"rm -rf "+ config_path +" \n";
    command +="tar -xf "+ file_save +" -C "+ Global::root_file +" \n";
    system(command.data());
    sync();
    return 0;
}

int Robot::writeRobotConfigParams()
{
    int ret=0;
    for(int i=0;i<AXIS_NUM;i++)
    {
        ret=axises[i]->servoParamListDownload();
        if(ret!=0)
            return ret;
    }
    return 0;
}

int Robot::rt4msPeriod()
{
    static double next_pos;
    static bool dir=true;

    //PDO Data is copied from bus to robot and tcp memory
    for(int i=0;i<AXIS_NUM;i++)
    {
        axises[i]->inputUpdate();
    }
    this->tcp_board->inputUpdate();


    //If the joint download parameters will be converted to disable state
    //The joint 6 will be enabled after it reaches the disable state.
    if (axises[5]->getAxisStatus()==NCAxis::NCAxisStatus::DISABLE)
    {
        if (!axises[5]->enable())
        {
            axises[5]->setMotionMode(NCAxis::MotionMode::CYCLIC_SYNCHRONOUS_POSITION);
            //Synchronize the joint output position to the current position before enabling successfully
            double pos = axises[5]->getPosition();
            axises[5]->setPosition(pos);
            next_pos=axises[5]->getPosition();
        }
    }
    //After the upper enable is successful, the axle will slowly rotate back and forth.
    else if (axises[5]->getAxisStatus()==NCAxis::NCAxisStatus::OPERATIONAL)
    {
        if(next_pos<1.5 && dir)
            next_pos+=0.0001;
        else if(next_pos>=1.5)
            dir=false;
        if(next_pos>0 && !dir)
            next_pos-=0.0001;
        else if(next_pos<0)
            dir=true;
        axises[5]->setPosition(next_pos);
    }

    //Copy axis and tcp memory data to PDO data
    for(int i=0;i<AXIS_NUM;i++)
    {
        axises[i]->outputSend();
    }
    this->tcp_board->outputSend();
}

int Robot::parseAxisConfigParamsFromJson()
{
    double pos_bias[AXIS_NUM];
    rapidjson::Document dom_bias;
    dom_bias.SetObject();
    std::string bias_json=Global::root_file+"/config/KinematicCalibration.json";
    std::ifstream in_bias(bias_json.data());
    if (!in_bias.is_open()) {
        return -1;
    }
    std::string json_content_bias((std::istreambuf_iterator<char>(in_bias)), std::istreambuf_iterator<char>());
    if (dom_bias.Parse(json_content_bias.c_str()).HasParseError())
        return -1;
    int count=dom_bias["links_kinematic_calibration"].GetArray().Size();
    for(int i=0;i<count;i++)
    {
        pos_bias[i]=dom_bias["links_kinematic_calibration"].GetArray()[i]["pos_bias"].GetDouble();
    }

    rapidjson::Document dom;
    dom.SetObject();
    std::string device_json=Global::root_file+"/config/device.json";
    std::ifstream in(device_json.data());
    if (!in.is_open()) {
        return -1;
    }
    std::string json_content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (dom.Parse(json_content.c_str()).HasParseError())
        return -1;
    if(dom["joint_list"].GetArray().Size()!=AXIS_NUM)
        return -1;
    for(int i=0;i<dom["joint_invert"].GetArray().Size();i++)
    {
        axises[i]->joint_invert=dom["joint_invert"][i].GetBool();
    }


    if(dom["joint_list"].GetArray().Size()!=AXIS_NUM)
        return -1;
    for(int i=0;i<dom["joint_invert"].GetArray().Size();i++)
    {
        axises[i]->joint_invert=dom["joint_invert"][i].GetBool();
    }

    for(int i=0;i<dom["joint_list"].GetArray().Size();i++)
    {
        std::string joint_name=dom["joint_list"].GetArray()[i].GetString();
        rapidjson::Document dom_axis;
        dom_axis.SetObject();
        std::string axis_json=Global::root_file+"/config/"+joint_name;
        std::ifstream in_axis(axis_json.data());
        if (!in_axis.is_open()) {
            return -1;
        }
        std::string json_content_axis((std::istreambuf_iterator<char>(in_axis)), std::istreambuf_iterator<char>());
        if (dom_axis.Parse(json_content_axis.c_str()).HasParseError())
            return -1;
        axises[i]->joint_model=dom_axis["joint_model"].GetString();
        int ABSencoderbits; /**< TODO: describe */
        int INCencoderlines;
        double frequencymulti; /**< TODO: describe */
        double ratio; /**< TODO: describe */


        double ratedtorque; /**< TODO: describe */
        double maxtorque; /**< TODO: describe */
        double ratedcurrent; /**< TODO: describe */
        double maxcurrent; /**< TODO: describe */
        double ratedspeed; /**< TODO: describe */
        double kt; /**< TODO: describe */
        double inertia; /**< TODO: describe */
        double brakemaxtorque; /**< TODO: describe */
        double brakequalifiedtorque; /**< TODO: describe */
        double brakequalifiedtorquefactor;  /**< TODO: describe */

        double stop1speedtolerance; /**< TODO: describe */
        double posrange[2]; /**< TODO: describe */
        double max_pos_err; /**< max pos error, unit rad */
        uint8_t motor_pole_pair;
        uint8_t stator_slots;
        ABSencoderbits=dom_axis["ABSencoderbits"].GetInt();
        INCencoderlines=dom_axis["INCencoderlines"].GetInt();
        frequencymulti=dom_axis["frequencymulti"].GetDouble();
        ratio=dom_axis["ratio"].GetDouble();

        ratedcurrent=dom_axis["ratedcurrent"].GetDouble();
        maxcurrent=dom_axis["maxcurrent"].GetDouble();
        ratedspeed=dom_axis["ratedspeed"].GetDouble();
        kt=dom_axis["kt"].GetDouble();
        inertia=dom_axis["inertia"].GetDouble();
        brakemaxtorque=dom_axis["brakemaxtorque"].GetDouble();
        brakequalifiedtorque=dom_axis["brakequalifiedtorque"].GetDouble();
        if (dom_axis.HasMember("brakequalifiedtorquefactor"))
        {
            brakequalifiedtorquefactor = dom_axis["brakequalifiedtorquefactor"].GetDouble();
        }
        else
        {
            brakequalifiedtorquefactor = 0.8;
        }
        if (dom_axis.HasMember("stop1speedtolerance"))
        {
            stop1speedtolerance = dom_axis["stop1speedtolerance"].GetDouble();
        }
        if (dom_axis.HasMember("motor_pole_pair"))
        {
            motor_pole_pair = dom_axis["motor_pole_pair"].GetInt();
        }
        else
        {
            motor_pole_pair = 8;
        }

        if (dom_axis.HasMember("stator_slots"))
        {
            stator_slots = dom_axis["stator_slots"].GetInt();

        }
        else
        {
            stator_slots = 18;
        }

        for(int i=0;i<2;i++)
        {
            posrange[i]=dom_axis["posrange"].GetArray()[i].GetDouble();
        }

        if (dom_axis.HasMember("max_pos_err"))
        {
            max_pos_err = dom_axis["max_pos_err"].GetDouble();
        }
        else
        {
            max_pos_err = 0.1745;
        }
        std::vector<NCAxis::ServoParam >p_s;
        for(int i=0;i<dom_axis["servo_list"].GetArray().Size();i++)
        {
            NCAxis::ServoParam servo;
            servo.name=dom_axis["servo_list"][i]["name"].GetString();
            servo.id=dom_axis["servo_list"][i]["id"].GetInt();
            servo.value=dom_axis["servo_list"][i]["value"].GetDouble();
            servo.qFmt=dom_axis["servo_list"][i]["qfmt"].GetDouble();
            p_s.push_back(servo);
        }
        axises[i]->setServoParamList(p_s);
        NCAxis::EncoderParam encoder_param;
        NCAxis::EncoderParam auxiliary_encoder_param;

        encoder_param.invert=axises[i]->joint_invert;
        encoder_param.scalefactor=2*(DucoCobot::PI)/pow(2,ABSencoderbits);
        encoder_param.posbias=pos_bias[i];

        axises[i]->setPrimaryEncoderParam(encoder_param);


        auxiliary_encoder_param.invert=axises[i]->joint_invert;
        auxiliary_encoder_param.scalefactor=2*(DucoCobot::PI)/INCencoderlines/frequencymulti/ratio;

        axises[i]->setAuxiliaryEncoderParam(auxiliary_encoder_param);

        NCAxis::MotorParam motor_param;
        motor_param.inertia=inertia;
        motor_param.ratedcurrent=ratedcurrent;
        motor_param.maxcurrent=maxcurrent;
        motor_param.kt=kt;
        motor_param.invert=axises[i]->joint_invert;
        motor_param.stator_slots=stator_slots;
        motor_param.pole_pair_num=motor_pole_pair;

        axises[i]->setMotorParam(motor_param);


        NCAxis::JointParam joint_param;
        joint_param.rated_speed=ratedspeed;

        if(dom_axis.HasMember("ratedtorque"))
        {
            ratedtorque=dom_axis["ratedtorque"].GetDouble();
            joint_param.rated_torque=ratedtorque;

        }
        if(dom_axis.HasMember("maxtorque"))
        {
            maxtorque=dom_axis["maxtorque"].GetDouble();
            joint_param.max_torque=maxtorque;

        }

        axises[i]->setJointParam(joint_param);

        NCAxis::BrakeParam break_param;
        break_param.max_torque=brakemaxtorque;
        break_param.qualified_torque=brakequalifiedtorque;
        break_param.qualified_torque_factor=brakequalifiedtorquefactor;

        axises[i]->setBrakeParam(break_param);

    }
}


void Robot::setEcMaster(EcMaster *master)
{
    this->master=master;
}

std::vector<AxisSafeControl *> Robot::getAxises()
{
    return axises;
}




}
