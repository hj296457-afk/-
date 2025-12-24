#ifndef CONTROL_H
#define CONTROL_H
#include <pthread.h>
#include <string>
#include <time.h>
namespace DucoCobot {
class Robot;
class EcMaster;
/**
 * @brief Robot start control
 *
 */
class Control
{
public:
    /**
     * @brief robot run
     *
     * @return int
     */
    int run();

    /**
     * @brief Enable real-time control thread, default period is 4 milliseconds
     *
     * @return int
     */
    int startRtPeriod();

    /**
     * @brief Real-time cycle thread function
     *
     * @param user
     */
    static void *periodFun(void *user);

    /**
     * @brief Set master object
     *
     * @param master
     */
    void setEcMaster(EcMaster* master);

    /**
     * @brief Set robot object
     *
     * @param robot
     */
    void setRobot(Robot* robot);

    /**
     * @brief set real-time cycle
     *
     * @param period
     */
    void setPeriod(int period);


    bool need_update_robot_params=false; /**< Do need to upload robot parameters? */


private:
    Robot* robot; /**< robot */
    EcMaster * master; /**< master */
    pthread_t *th; /**< pthread* */
    unsigned int period=4000000;/**<  real-time control, default period is 4 milliseconds */
    timespec period_time; /**< The next wake-up cycle time */
    bool stop=false; /**<  */

};


}


#endif
