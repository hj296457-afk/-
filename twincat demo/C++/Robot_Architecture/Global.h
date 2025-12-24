#ifndef GLOBAL_H
#define GLOBAL_H
#include <unistd.h>
#include <string>

namespace DucoCobot {
/**
 * @brief Global Settings
 *
 */
class Global
{
public:
//Number of axes
#define AXIS_NUM 6
#define NO_MOTION_CMD_VEL_TOL 550
#define MIN_REDUCER_TEMPERATURE 25
#define MAX_REDUCER_TEMPERATURE 50
#define ARM_CYCLE_TIME 0.004

static std::string root_file; /**< Configuration file root path*/


};

}

#endif
