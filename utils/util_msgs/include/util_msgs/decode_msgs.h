#ifndef __util_msgs_util_msgs_H__
#define __util_msgs_util_msgs_H__

#include <stdint.h>
#include <vector>
#include <util_msgs/OutputData.h>
#include <util_msgs/StatusData.h>
#include <util_msgs/PPROutputData.h>

namespace util_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      util_msgs::OutputData &output);

bool decodeStatusData(const std::vector<uint8_t> &data,
                      util_msgs::StatusData &status);

bool decodePPROutputData(const std::vector<uint8_t> &data,
                         util_msgs::PPROutputData &output);
}

#endif
