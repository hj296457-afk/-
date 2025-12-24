#include "Control.h"
#include "EcMaster/EcMaster.h"
#include "Robot/Robot.h"
#include <unistd.h>
#include <iostream>
#include "string.h"

namespace DucoCobot {

int Control::run()
{
    int ret=0;
    if(!master || !robot)
        return -1;

    //init master
    master->init();

    //Start the real-time cycle control thread
    this->startRtPeriod();
    sleep(2);

    //master config
    ret=master->configMaster();
    if(ret!=0)
        return -1;
    sleep(2);

    //Do need to upload robot parameters?
    if(need_update_robot_params)
    {
        //Upload robot parameters from tcp
        ret=robot->uploadFileFromTcpBoard();
        if(ret!=0)
            return ret;
        sleep(2);
    }

    //Robot parameter analysis
    ret=robot->parseAxisConfigParamsFromJson();
    if(ret!=0)
        return ret;

    sleep(2);

    //Configure the master station to enter the op
    ret=master->setMasterState(8);
    if(ret!=0)
        return -1;

    sleep(5);

    //Download servo parameters to joints through sdo
    robot->writeRobotConfigParams();

}


int Control::startRtPeriod()
{

    th=new pthread_t;
    pthread_attr_t      ap;
    struct sched_param  sp;
    cpu_set_t           cp;
    memset(&ap,0,sizeof(pthread_attr_t));
    memset(&sp,0,sizeof(struct sched_param));
    memset(&cp,0,sizeof(cpu_set_t));
    pthread_attr_init(&ap);
    pthread_attr_setinheritsched(&ap, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&ap, SCHED_FIFO);  // 调度策略为FIFO
    sp.sched_priority = 99;                        // 优先级为90
    pthread_attr_setschedparam(&ap, &sp);
    CPU_ZERO(&cp);
    CPU_SET(2, &cp);                               // 绑定在CPU#1核上
    pthread_attr_setaffinity_np(&ap, sizeof(cp), &cp);
    pthread_create(th, &ap, periodFun, (void*)(this));
    clock_gettime(CLOCK_MONOTONIC, &period_time);

}

void* Control::periodFun(void* user)
{
    Control* control=static_cast<Control*> (user);
    while(!control->stop)
    {
        static bool first_period=true;

        control->period_time.tv_nsec+=control->period;
        while (control->period_time.tv_nsec >= 1000000000) {
            control->period_time.tv_nsec -= 1000000000;
            control->period_time.tv_sec++;
        }
        //Block until wake-up time
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &control->period_time, NULL);
        if(first_period && control->master)
        {
            control->master->writeToBus();
            first_period=false;
        }else if(control->master){
            control->master->readFromBus();
            control->master->writeToBus();
            control->robot->rt4msPeriod();
        }
    }
}


void Control::setEcMaster(EcMaster *master)
{
    this->master=master;
}


void Control::setRobot(Robot *robot)
{
    this->robot=robot;
}


void Control::setPeriod(int period)
{
    this->period=period;
}


























}
