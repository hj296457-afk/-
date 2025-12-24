#ifndef ROBOT_H
#define ROBOT_H
#include <vector>
#include "NCAxis.h"
namespace DucoCobot {
class EcMaster;
class AxisControl;
class AxisSafeControl;
class TcpControl;
/**
 * @brief Robot abstract
 *
 */
class Robot
{
public:
    /**
     * @brief
     *
     * @param master
     */
    Robot(EcMaster* master);
    /**
     * @brief Robot initialization
     *
     * @return int
     */
    int init();


    /**
     * @brief Upload robot parameters from tcp
     *
     * @return int
     */
    int uploadFileFromTcpBoard();

    /**
     * @brief  Download joint parameters to joint
     *
     * @return int
     */
    int writeRobotConfigParams();

    /**
     * @brief  Parse joint configuration parameters from configuration file
     *
     * @return int
     */
    int parseAxisConfigParamsFromJson();


    /**
     * @brief
     *
     * @param master
     */
    void setEcMaster(EcMaster* master);

    /**
     * @brief
     *
     * @return std::vector<AxisSafeControl *>
     */
    std::vector<AxisSafeControl*> getAxises();

    /**
     * @brief Robot real-time cycle control
     *
     * @return int
     */
    int rt4msPeriod();
private:
    std::vector<AxisSafeControl*> axises;

    EcMaster *master;

    TcpControl* tcp_board; /**< tcp */

    int sdo_w_index_ = 0x2020;  /**< local servo sdo write channel index */
    int sdo_w_subindex_ = 0;
    int sdo_r_index_ = 0x2030;  /**< local servo sdo read channel index */
    int sdo_r_subindex_ = 0;
};



}

#endif
