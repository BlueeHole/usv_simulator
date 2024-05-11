#ifndef __util_msgs_util_msgs_H__
#define __util_msgs_util_msgs_H__

#include <stdint.h>
#include <vector>
#include <util_msgs/SO3Command.h>
#include <util_msgs/TRPYCommand.h>
#include <util_msgs/Gains.h>

namespace util_msgs
{

void encodeSO3Command(const util_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output);
void encodeTRPYCommand(const util_msgs::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);

void encodePPRGains(const util_msgs::Gains &gains,
                    std::vector<uint8_t> &output);
}

#endif
