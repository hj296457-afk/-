#include <unistd.h>
#include "Control/Control.h"
#include "EcMaster/EcMaster.h"
#include "Robot/Robot.h"
#include "Global.h"
#include "dirent.h"




using namespace std;
using namespace DucoCobot;

int main()
{
    //Create a robot control object
    Control *robot_control =new Control;

    //Create master object
    EcMaster *master;//=new EcMaster;

    //Create a robot object
    Robot* robot=new Robot(master);

    robot_control->setEcMaster(master);
    robot_control->setRobot(robot);

    //Robot control program running
    robot_control->run();
    while(1)
    {
        sleep(1);
    }
    return 1;
}
