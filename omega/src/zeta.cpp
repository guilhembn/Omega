#include "omega/zeta.hpp"
#include <sensor_msgs/image_encodings.h>
#include <math.h> 
#include <cv_bridge/cv_bridge.h>
#include <stdexcept>
#include <std_msgs/String.h>

const std::vector<std::vector<uint8_t>> Zeta::questions({
{0x34, 0x33, 0x36, 0x66, 0x36, 0x64, 0x36, 0x32, 0x36, 0x39, 0x36, 0x35, 0x36, 0x65, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x36, 0x36, 0x66, 0x36, 0x65, 0x36, 0x33, 0x37, 0x34, 0x36, 0x39, 0x36, 0x66, 0x36, 0x65, 0x36, 0x65, 0x36, 0x31, 0x36, 0x63, 0x36, 0x39, 0x37, 0x34, 0x63, 0x33, 0x61, 0x39, 0x37, 0x33, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, 0x32, 0x30, 0x36, 0x31, 0x32, 0x30, 0x34, 0x66, 0x37, 0x32, 0x36, 0x66, 0x32, 0x30, 0x36, 0x33, 0x36, 0x66, 0x36, 0x64, 0x37, 0x30, 0x36, 0x31, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x32, 0x30, 0x63, 0x33, 0x61, 0x30, 0x32, 0x30, 0x36, 0x66, 0x36, 0x65, 0x37, 0x34, 0x36, 0x66, 0x36, 0x63, 0x36, 0x66, 0x36, 0x37, 0x36, 0x35, 0x36, 0x65, 0x36, 0x39, 0x37, 0x35, 0x37, 0x33, 0x32, 0x30, 0x33, 0x66, },
{0x34, 0x33, 0x36, 0x66, 0x36, 0x64, 0x36, 0x32, 0x36, 0x39, 0x36, 0x35, 0x36, 0x65, 0x32, 0x30, 0x37, 0x39, 0x32, 0x30, 0x36, 0x31, 0x37, 0x36, 0x36, 0x31, 0x36, 0x39, 0x37, 0x34, 0x32, 0x64, 0x36, 0x39, 0x36, 0x63, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x35, 0x37, 0x32, 0x36, 0x63, 0x36, 0x35, 0x37, 0x33, 0x32, 0x30, 0x35, 0x32, 0x34, 0x39, 0x35, 0x33, 0x32, 0x30, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x37, 0x30, 0x36, 0x35, 0x37, 0x32, 0x37, 0x34, 0x36, 0x66, 0x37, 0x32, 0x36, 0x39, 0x63, 0x33, 0x61, 0x39, 0x36, 0x35, 0x37, 0x33, 0x32, 0x30, 0x36, 0x31, 0x37, 0x35, 0x32, 0x30, 0x33, 0x38, 0x32, 0x30, 0x36, 0x64, 0x36, 0x31, 0x36, 0x39, 0x32, 0x30, 0x33, 0x32, 0x33, 0x30, 0x33, 0x32, 0x33, 0x31, 0x32, 0x30, 0x33, 0x66, },
{0x35, 0x33, 0x37, 0x35, 0x37, 0x32, 0x32, 0x30, 0x36, 0x33, 0x36, 0x66, 0x36, 0x64, 0x36, 0x32, 0x36, 0x39, 0x36, 0x35, 0x36, 0x65, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x36, 0x36, 0x35, 0x37, 0x32, 0x37, 0x33, 0x36, 0x39, 0x36, 0x66, 0x36, 0x65, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x64, 0x36, 0x66, 0x37, 0x36, 0x36, 0x35, 0x35, 0x38, 0x34, 0x34, 0x32, 0x30, 0x34, 0x61, 0x37, 0x35, 0x36, 0x63, 0x36, 0x35, 0x37, 0x33, 0x32, 0x30, 0x36, 0x31, 0x32, 0x64, 0x37, 0x34, 0x32, 0x64, 0x36, 0x39, 0x36, 0x63, 0x32, 0x30, 0x37, 0x34, 0x37, 0x32, 0x36, 0x31, 0x37, 0x36, 0x36, 0x31, 0x36, 0x39, 0x36, 0x63, 0x36, 0x63, 0x63, 0x33, 0x61, 0x39, 0x32, 0x30, 0x33, 0x66, },
{0x35, 0x31, 0x37, 0x35, 0x36, 0x39, 0x32, 0x30, 0x36, 0x31, 0x32, 0x30, 0x36, 0x34, 0x36, 0x39, 0x37, 0x34, 0x32, 0x30, 0x65, 0x32, 0x38, 0x30, 0x39, 0x63, 0x34, 0x61, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x36, 0x36, 0x63, 0x33, 0x61, 0x38, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x63, 0x33, 0x61, 0x61, 0x37, 0x34, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x36, 0x33, 0x36, 0x31, 0x37, 0x30, 0x36, 0x39, 0x37, 0x34, 0x36, 0x31, 0x36, 0x63, 0x36, 0x39, 0x37, 0x33, 0x37, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x31, 0x37, 0x35, 0x36, 0x35, 0x32, 0x30, 0x37, 0x32, 0x36, 0x66, 0x37, 0x35, 0x37, 0x38, 0x65, 0x32, 0x38, 0x30, 0x39, 0x64, 0x32, 0x30, 0x33, 0x66, },
{0x35, 0x31, 0x37, 0x35, 0x36, 0x35, 0x36, 0x63, 0x32, 0x30, 0x36, 0x64, 0x36, 0x35, 0x36, 0x64, 0x36, 0x32, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x35, 0x32, 0x34, 0x39, 0x35, 0x33, 0x32, 0x30, 0x36, 0x31, 0x32, 0x30, 0x36, 0x34, 0x36, 0x66, 0x36, 0x65, 0x36, 0x65, 0x63, 0x33, 0x61, 0x39, 0x32, 0x30, 0x37, 0x33, 0x36, 0x66, 0x36, 0x65, 0x32, 0x30, 0x36, 0x65, 0x36, 0x66, 0x36, 0x64, 0x32, 0x30, 0x63, 0x33, 0x61, 0x30, 0x32, 0x30, 0x37, 0x35, 0x36, 0x65, 0x36, 0x35, 0x32, 0x30, 0x37, 0x34, 0x36, 0x35, 0x36, 0x33, 0x36, 0x38, 0x36, 0x65, 0x36, 0x39, 0x37, 0x31, 0x37, 0x35, 0x36, 0x35, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x34, 0x36, 0x31, 0x37, 0x32, 0x36, 0x66, 0x37, 0x34, 0x32, 0x30, 0x33, 0x66, },
{0x34, 0x34, 0x37, 0x35, 0x37, 0x32, 0x36, 0x31, 0x36, 0x65, 0x37, 0x34, 0x32, 0x30, 0x36, 0x63, 0x36, 0x31, 0x32, 0x30, 0x36, 0x33, 0x36, 0x31, 0x36, 0x63, 0x36, 0x39, 0x36, 0x32, 0x37, 0x32, 0x36, 0x31, 0x37, 0x34, 0x36, 0x39, 0x36, 0x66, 0x36, 0x65, 0x32, 0x30, 0x32, 0x38, 0x36, 0x31, 0x37, 0x30, 0x37, 0x32, 0x63, 0x33, 0x61, 0x38, 0x37, 0x33, 0x32, 0x30, 0x36, 0x63, 0x65, 0x32, 0x38, 0x30, 0x39, 0x39, 0x36, 0x31, 0x36, 0x63, 0x36, 0x63, 0x37, 0x35, 0x36, 0x64, 0x36, 0x31, 0x36, 0x37, 0x36, 0x35, 0x32, 0x39, 0x32, 0x30, 0x36, 0x34, 0x37, 0x35, 0x32, 0x30, 0x35, 0x30, 0x35, 0x32, 0x33, 0x32, 0x32, 0x30, 0x36, 0x63, 0x36, 0x35, 0x37, 0x33, 0x32, 0x30, 0x36, 0x37, 0x37, 0x32, 0x36, 0x39, 0x37, 0x30, 0x37, 0x30, 0x36, 0x35, 0x37, 0x32, 0x37, 0x33, 0x32, 0x30, 0x37, 0x33, 0x36, 0x35, 0x32, 0x30, 0x36, 0x33, 0x36, 0x31, 0x36, 0x63, 0x36, 0x39, 0x36, 0x32, 0x37, 0x32, 0x36, 0x35, 0x36, 0x65, 0x37, 0x34, 0x32, 0x30, 0x36, 0x31, 0x37, 0x36, 0x36, 0x31, 0x36, 0x65, 0x37, 0x34, 0x32, 0x30, 0x36, 0x63, 0x36, 0x31, 0x32, 0x30, 0x37, 0x34, 0x63, 0x33, 0x61, 0x61, 0x37, 0x34, 0x36, 0x35, 0x32, 0x65, },
{0x35, 0x31, 0x37, 0x35, 0x36, 0x35, 0x36, 0x63, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x35, 0x36, 0x63, 0x37, 0x35, 0x36, 0x33, 0x36, 0x38, 0x36, 0x35, 0x32, 0x30, 0x36, 0x65, 0x65, 0x32, 0x38, 0x30, 0x39, 0x39, 0x36, 0x31, 0x32, 0x30, 0x36, 0x61, 0x36, 0x31, 0x36, 0x64, 0x36, 0x31, 0x36, 0x39, 0x37, 0x33, 0x32, 0x30, 0x63, 0x33, 0x61, 0x39, 0x37, 0x34, 0x63, 0x33, 0x61, 0x39, 0x32, 0x30, 0x37, 0x30, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x37, 0x33, 0x36, 0x35, 0x36, 0x65, 0x37, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x34, 0x36, 0x31, 0x36, 0x65, 0x37, 0x33, 0x32, 0x30, 0x36, 0x63, 0x32, 0x37, 0x36, 0x66, 0x37, 0x30, 0x36, 0x35, 0x36, 0x65, 0x32, 0x30, 0x37, 0x33, 0x37, 0x30, 0x36, 0x31, 0x36, 0x33, 0x36, 0x35, 0x32, 0x30, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, 0x32, 0x30, 0x63, 0x33, 0x61, 0x30, 0x32, 0x30, 0x36, 0x63, 0x32, 0x37, 0x34, 0x35, 0x37, 0x33, 0x37, 0x34, 0x32, 0x30, 0x33, 0x66, },
{0x34, 0x34, 0x37, 0x35, 0x37, 0x32, 0x36, 0x31, 0x36, 0x65, 0x37, 0x34, 0x32, 0x30, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x37, 0x33, 0x63, 0x33, 0x61, 0x39, 0x36, 0x64, 0x36, 0x39, 0x36, 0x65, 0x36, 0x31, 0x36, 0x39, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x35, 0x32, 0x34, 0x39, 0x35, 0x33, 0x32, 0x30, 0x33, 0x32, 0x33, 0x30, 0x33, 0x31, 0x33, 0x38, 0x32, 0x63, 0x32, 0x30, 0x36, 0x63, 0x36, 0x31, 0x32, 0x30, 0x37, 0x34, 0x36, 0x31, 0x36, 0x32, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x36, 0x34, 0x65, 0x32, 0x38, 0x30, 0x39, 0x39, 0x34, 0x31, 0x36, 0x64, 0x36, 0x31, 0x36, 0x65, 0x36, 0x34, 0x36, 0x39, 0x36, 0x65, 0x36, 0x35, 0x32, 0x30, 0x36, 0x35, 0x37, 0x33, 0x37, 0x34, 0x32, 0x30, 0x36, 0x31, 0x37, 0x32, 0x37, 0x32, 0x36, 0x39, 0x37, 0x36, 0x63, 0x33, 0x61, 0x39, 0x36, 0x35, 0x32, 0x30, 0x36, 0x35, 0x36, 0x65, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x37, 0x35, 0x37, 0x38, 0x36, 0x39, 0x63, 0x33, 0x61, 0x38, 0x36, 0x64, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x66, 0x37, 0x33, 0x36, 0x39, 0x37, 0x34, 0x36, 0x39, 0x36, 0x66, 0x36, 0x65, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x63, 0x36, 0x31, 0x32, 0x30, 0x37, 0x34, 0x36, 0x31, 0x36, 0x32, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x36, 0x31, 0x37, 0x39, 0x36, 0x31, 0x36, 0x65, 0x37, 0x34, 0x32, 0x30, 0x36, 0x64, 0x36, 0x31, 0x36, 0x65, 0x36, 0x37, 0x63, 0x33, 0x61, 0x39, 0x32, 0x30, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x32, 0x36, 0x31, 0x36, 0x33, 0x36, 0x63, 0x36, 0x35, 0x37, 0x34, 0x37, 0x34, 0x36, 0x35, 0x32, 0x65, },
{0x34, 0x33, 0x36, 0x66, 0x36, 0x64, 0x36, 0x32, 0x36, 0x39, 0x36, 0x35, 0x36, 0x65, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x37, 0x36, 0x36, 0x35, 0x37, 0x32, 0x37, 0x33, 0x36, 0x39, 0x36, 0x66, 0x36, 0x65, 0x37, 0x33, 0x32, 0x30, 0x36, 0x35, 0x37, 0x38, 0x37, 0x30, 0x36, 0x63, 0x36, 0x39, 0x37, 0x31, 0x37, 0x35, 0x36, 0x31, 0x36, 0x65, 0x37, 0x34, 0x32, 0x30, 0x36, 0x63, 0x36, 0x31, 0x32, 0x30, 0x36, 0x33, 0x36, 0x66, 0x37, 0x35, 0x36, 0x63, 0x36, 0x35, 0x37, 0x35, 0x37, 0x32, 0x32, 0x30, 0x37, 0x36, 0x36, 0x35, 0x37, 0x32, 0x37, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x63, 0x36, 0x31, 0x32, 0x30, 0x36, 0x32, 0x36, 0x39, 0x63, 0x33, 0x61, 0x38, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x36, 0x32, 0x37, 0x35, 0x36, 0x35, 0x32, 0x30, 0x63, 0x33, 0x61, 0x30, 0x32, 0x30, 0x35, 0x34, 0x36, 0x31, 0x36, 0x64, 0x37, 0x30, 0x36, 0x35, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x36, 0x63, 0x36, 0x35, 0x32, 0x30, 0x37, 0x33, 0x36, 0x35, 0x37, 0x32, 0x37, 0x36, 0x36, 0x35, 0x37, 0x35, 0x37, 0x32, 0x32, 0x30, 0x36, 0x31, 0x32, 0x64, 0x37, 0x34, 0x32, 0x64, 0x36, 0x39, 0x36, 0x63, 0x32, 0x30, 0x36, 0x34, 0x36, 0x66, 0x36, 0x65, 0x36, 0x65, 0x63, 0x33, 0x61, 0x39, 0x32, 0x30, 0x33, 0x66, },
{0x35, 0x31, 0x37, 0x35, 0x36, 0x35, 0x32, 0x30, 0x36, 0x36, 0x36, 0x31, 0x37, 0x35, 0x37, 0x34, 0x32, 0x64, 0x36, 0x39, 0x36, 0x63, 0x32, 0x30, 0x36, 0x36, 0x36, 0x31, 0x36, 0x39, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x36, 0x34, 0x65, 0x32, 0x38, 0x30, 0x39, 0x39, 0x36, 0x31, 0x37, 0x30, 0x37, 0x32, 0x63, 0x33, 0x61, 0x38, 0x37, 0x33, 0x32, 0x30, 0x35, 0x32, 0x36, 0x31, 0x36, 0x33, 0x36, 0x38, 0x36, 0x39, 0x36, 0x34, 0x32, 0x30, 0x33, 0x66, },
});

const std::vector<std::vector<std::vector<uint8_t>>> Zeta::answers({
{	{0x33, 0x35, 0x32, 0x30, 0x36, 0x36, 0x36, 0x66, 0x36, 0x39, 0x37, 0x33, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, },
	{0x33, 0x31, 0x33, 0x30, 0x32, 0x30, 0x36, 0x36, 0x36, 0x66, 0x36, 0x39, 0x37, 0x33, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, },
	{0x33, 0x32, 0x33, 0x30, 0x32, 0x30, 0x36, 0x36, 0x36, 0x66, 0x36, 0x39, 0x37, 0x33, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, },
	{0x33, 0x31, 0x33, 0x30, 0x33, 0x30, 0x32, 0x30, 0x36, 0x36, 0x36, 0x66, 0x36, 0x39, 0x37, 0x33, 0x32, 0x30, 0x37, 0x30, 0x36, 0x63, 0x37, 0x35, 0x37, 0x33, },
},
{	{0x33, 0x34, 0x33, 0x32, },
	{0x33, 0x35, 0x33, 0x31, },
	{0x33, 0x36, 0x33, 0x39, },
	{0x33, 0x31, 0x33, 0x30, 0x33, 0x31, },
},
{	{0x33, 0x32, },
	{0x33, 0x33, },
	{0x33, 0x34, },
	{0x33, 0x35, },
},
{	{0x34, 0x34, 0x36, 0x31, 0x37, 0x36, 0x36, 0x39, 0x36, 0x34, },
	{0x34, 0x61, 0x37, 0x35, 0x36, 0x63, 0x36, 0x35, 0x37, 0x33, },
	{0x34, 0x37, 0x37, 0x35, 0x36, 0x39, 0x36, 0x63, 0x36, 0x63, 0x36, 0x31, 0x37, 0x35, 0x36, 0x64, 0x36, 0x35, },
	{0x34, 0x31, 0x36, 0x64, 0x36, 0x31, 0x36, 0x65, 0x36, 0x34, 0x36, 0x39, 0x36, 0x65, 0x36, 0x35, },
},
{	{0x34, 0x31, 0x36, 0x64, 0x63, 0x33, 0x61, 0x39, 0x36, 0x63, 0x36, 0x39, 0x36, 0x35, },
	{0x34, 0x31, 0x36, 0x64, 0x36, 0x31, 0x36, 0x65, 0x36, 0x34, 0x36, 0x39, 0x36, 0x65, 0x36, 0x35, },
	{0x34, 0x37, 0x37, 0x35, 0x36, 0x39, 0x36, 0x63, 0x36, 0x63, 0x36, 0x31, 0x37, 0x35, 0x36, 0x64, 0x36, 0x35, },
	{0x35, 0x32, 0x36, 0x31, 0x36, 0x36, 0x36, 0x31, },
},
{	{0x35, 0x36, 0x37, 0x32, 0x36, 0x31, 0x36, 0x39, },
	{0x34, 0x36, 0x36, 0x31, 0x37, 0x35, 0x37, 0x38, },
},
{	{0x35, 0x35, 0x36, 0x65, 0x32, 0x30, 0x36, 0x65, 0x36, 0x35, 0x37, 0x35, 0x37, 0x32, 0x36, 0x66, 0x36, 0x65, 0x36, 0x35, },
	{0x35, 0x35, 0x36, 0x65, 0x36, 0x35, 0x32, 0x30, 0x36, 0x33, 0x36, 0x31, 0x37, 0x32, 0x36, 0x66, 0x37, 0x34, 0x37, 0x34, 0x36, 0x35, },
	{0x35, 0x35, 0x36, 0x65, 0x36, 0x35, 0x32, 0x30, 0x36, 0x32, 0x36, 0x31, 0x36, 0x63, 0x36, 0x35, 0x36, 0x39, 0x36, 0x65, 0x36, 0x35, },
	{0x35, 0x35, 0x36, 0x65, 0x32, 0x30, 0x36, 0x63, 0x36, 0x66, 0x37, 0x35, 0x37, 0x30, },
},
{	{0x35, 0x36, 0x37, 0x32, 0x36, 0x31, 0x36, 0x39, },
	{0x34, 0x36, 0x36, 0x31, 0x37, 0x35, 0x37, 0x38, },
},
{	{0x35, 0x35, 0x36, 0x65, 0x36, 0x35, },
	{0x34, 0x34, 0x36, 0x35, 0x37, 0x35, 0x37, 0x38, },
	{0x35, 0x34, 0x37, 0x32, 0x36, 0x66, 0x36, 0x39, 0x37, 0x33, },
	{0x35, 0x31, 0x37, 0x35, 0x36, 0x31, 0x37, 0x34, 0x37, 0x32, 0x36, 0x35, },
},
{	{0x35, 0x30, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x36, 0x33, 0x36, 0x39, 0x37, 0x33, 0x36, 0x35, 0x37, 0x32, 0x32, 0x30, 0x36, 0x35, 0x37, 0x34, 0x32, 0x30, 0x37, 0x32, 0x36, 0x31, 0x36, 0x61, 0x36, 0x66, 0x37, 0x35, 0x37, 0x34, 0x36, 0x35, 0x37, 0x32, },
	{0x34, 0x31, 0x37, 0x32, 0x36, 0x37, 0x37, 0x35, 0x36, 0x64, 0x36, 0x35, 0x36, 0x65, 0x37, 0x34, 0x36, 0x35, 0x37, 0x32, 0x32, 0x30, 0x36, 0x35, 0x37, 0x34, 0x32, 0x30, 0x36, 0x31, 0x36, 0x37, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x36, 0x64, 0x36, 0x35, 0x36, 0x65, 0x37, 0x34, 0x36, 0x35, 0x37, 0x32, },
	{0x34, 0x31, 0x36, 0x36, 0x36, 0x36, 0x36, 0x39, 0x36, 0x65, 0x36, 0x35, 0x37, 0x32, 0x32, 0x30, 0x36, 0x35, 0x37, 0x34, 0x32, 0x30, 0x36, 0x35, 0x36, 0x65, 0x37, 0x32, 0x36, 0x39, 0x36, 0x33, 0x36, 0x38, 0x36, 0x39, 0x37, 0x32, },
	{0x35, 0x34, 0x37, 0x32, 0x36, 0x31, 0x37, 0x36, 0x36, 0x31, 0x36, 0x39, 0x36, 0x63, 0x36, 0x63, 0x36, 0x35, 0x37, 0x32, 0x32, 0x30, 0x36, 0x39, 0x36, 0x65, 0x36, 0x33, 0x37, 0x32, 0x63, 0x33, 0x61, 0x39, 0x36, 0x64, 0x36, 0x35, 0x36, 0x65, 0x37, 0x34, 0x36, 0x31, 0x36, 0x63, 0x36, 0x35, 0x36, 0x64, 0x36, 0x35, 0x36, 0x65, 0x37, 0x34, 0x32, 0x63, 0x32, 0x30, 0x63, 0x33, 0x61, 0x61, 0x37, 0x34, 0x37, 0x32, 0x36, 0x35, 0x32, 0x30, 0x36, 0x64, 0x36, 0x66, 0x36, 0x34, 0x36, 0x35, 0x37, 0x33, 0x37, 0x34, 0x36, 0x35, 0x32, 0x30, 0x36, 0x35, 0x37, 0x34, 0x32, 0x30, 0x36, 0x65, 0x36, 0x35, 0x32, 0x30, 0x37, 0x30, 0x36, 0x31, 0x37, 0x33, 0x32, 0x30, 0x37, 0x36, 0x36, 0x39, 0x37, 0x33, 0x36, 0x35, 0x37, 0x32, 0x32, 0x30, 0x37, 0x34, 0x37, 0x32, 0x36, 0x66, 0x37, 0x30, 0x32, 0x30, 0x36, 0x63, 0x36, 0x66, 0x36, 0x39, 0x36, 0x65, 0x32, 0x30, 0x37, 0x34, 0x37, 0x32, 0x36, 0x66, 0x37, 0x30, 0x32, 0x30, 0x37, 0x36, 0x36, 0x39, 0x37, 0x34, 0x36, 0x35, },
},
});

const std::vector<size_t> Zeta::cor({
    1,2,1,0,3,0,3,1,1,2
});

const std::vector<uint8_t> Zeta::sec({
    0x36, 0x38, 0x37, 0x34, 0x37, 0x34, 0x37, 0x30, 0x37, 0x33, 0x33, 0x61, 0x32, 0x66, 0x32, 0x66, 0x36, 0x38, 0x36, 0x66, 0x36, 0x64, 0x36, 0x35, 0x37, 0x30, 0x36, 0x31, 0x36, 0x37, 0x36, 0x35, 0x37, 0x33, 0x32, 0x65, 0x36, 0x63, 0x36, 0x31, 0x36, 0x31, 0x37, 0x33, 0x32, 0x65, 0x36, 0x36, 0x37, 0x32, 0x32, 0x66, 0x36, 0x37, 0x36, 0x32, 0x37, 0x35, 0x36, 0x39, 0x37, 0x33, 0x36, 0x31, 0x36, 0x65, 0x32, 0x66, 0x36, 0x33, 0x37, 0x35, 0x36, 0x63, 0x37, 0x34, 0x37, 0x35, 0x37, 0x32, 0x36, 0x35,
});

Zeta::Zeta(ros::NodeHandlePtr nh): Enigma(nh), headact_("/head_traj_controller/point_head_action"), it_(*nh_), capture_(false), lastAnswer_(' '){
    imgSub_ = it_.subscribe("/wide_stereo/left/image_rect", 1, &Zeta::imageCb, this);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    sayPub_ = nh->advertise<std_msgs::String>("/say", 5);
    headact_.waitForServer(ros::Duration(5.0));
    if (!headact_.isServerConnected()){
        std::cout << "Quelque chose ne va pas... Lancez vous bien cet executable sur le PR2 ? Le PR2 est-il prêt (i.e. robot.launch lancé) ?" << std::endl;
        throw std::runtime_error("Arrêt du système Omega.");
    }
}

std::string Zeta::name(){
    return "Zeta";
}

void Zeta::run(){
    if (!waitForRunStop(5.0)){
        if (!isRunStopEnabled_){
            std::cout << "Merci d'activer le 'run/stop' du robot." << std::endl;
            throw std::runtime_error("Arrêt du système Omega");
        }
    }
    look();
    char L[4] = {'A', 'B', 'C', 'D'};
    std::vector<std::tuple<std::string, std::vector<std::string>, char>> qs;
    for (size_t i = 0; i < questions.size(); i++){
        std::vector<std::string> ans;
        for (const auto &a: answers[i]){
            ans.push_back(demangleString(a));
        }
        qs.push_back(std::make_tuple(demangleString(questions[i]), ans, L[cor[i]]));
    }    
    for (const auto &p: qs){
        capture_ = false;
        lastAnswer_ = ' ';
        std::string question = std::get<0>(p);
        std::cout << question << std::endl;
        say(question);
        for (size_t i=0; i < std::get<1>(p).size(); i++){
            std::cout << L[i] << " - " << std::get<1>(p)[i] <<std::endl;
            say(std::string(1, L[i]) + ", " + std::get<1>(p)[i]);
            ros::Duration(0.75).sleep();
        }
        for (unsigned int i=5; i>0; i--){
            std::cout << "\rValidation de la réponse dans " << i << " secondes" << std::flush;
            ros::Duration(1.0).sleep();
        }
        std::cout << std::endl;
        lastAnswer_ = ' ';
        capture_ = true;
        for (unsigned int i=0; i<100; i++){
            if (lastAnswer_ != ' '){
                break;
            }
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        capture_ = false;
        if (lastAnswer_ == ' '){
            std::cout << "Pas de réponse détectée, fin de la procédure." << std::endl;
            return;
        }
        assert(lastAnswer_ == 'A' || lastAnswer_ == 'B' || lastAnswer_ == 'C' || lastAnswer_ == 'D');
        std::cout << "Votre réponse : " << lastAnswer_ << std::endl;
        if (lastAnswer_ != std::get<2>(p)){
            std::cout << "Mauvaise réponse ! Fin de la procédure." << std::endl;
            say("Mauvaise réponse ! Fin de la procédure.");
            return;
        }else{
            std::cout << "Réponse valide." << std::endl;
            say("Réponse valide.");
        }
    }
    say("Félicitations ! La suite se trouve dans le terminal.");
    std::cout << "Félicitations !" << std::endl << demangleString(sec) << std::endl << "Fin de la procédure." << std::endl;
}

void Zeta::imageCb(const sensor_msgs::ImageConstPtr& msg){
    if (!capture_){
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, markerCorners, markerIds, parameters, rejectedCandidates);
    int index = -1;
    for (size_t i=0; i < markerIds.size(); i++){
        int markerId = markerIds[i];
        if (markerId == 13){
            index = i;
            break;
        }
    }
    if (index == -1){
        return;
    }
    float angle = getAngle(markerCorners[index]);
    if (angle <= 3*M_PI/4 && angle > M_PI/4){
        //std::cout << "B" << std::endl;
        lastAnswer_ = 'B';
    }else if (angle <= M_PI/4 && angle > -M_PI/4){
        //std::cout << "A" << std::endl;
        lastAnswer_ = 'A';
    }else if (angle <= -M_PI/4 && angle > -3*M_PI/4){
        //std::cout << "D" << std::endl;
        lastAnswer_ = 'D';
    }else{
        //std::cout << "C" << std::endl;
        lastAnswer_ = 'C';
    }
}



float Zeta::getAngle(const std::vector<cv::Point2f>& markerCorners) const{
    cv::Point2f topBary = (markerCorners[0] + markerCorners[1]) / 2.0f;
    cv::Point2f bottomBary = (markerCorners[2] + markerCorners[3]) / 2.0f;
    cv::Point2f orientVec = topBary - bottomBary;
    return centerAngle(atan2f(orientVec.y, orientVec.x) + 1.57079632679);
}

float Zeta::centerAngle(float angle){
    while (angle < -M_PI){
        angle += 2*M_PI;
    }
    while (angle >= M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}

void Zeta::say(const std::string& speech){
    std_msgs::String msg;
    msg.data = speech;
    sayPub_.publish(msg);
    ros::Duration(0.55 * (std::count(speech.cbegin(), speech.cend(), ' ') + 1)).sleep();
}

std::string Zeta::demangleString(const std::vector<uint8_t>& vec){
    std::string str(vec.begin(), vec.end());
    std::string output;

    if ((str.length() % 2) != 0) {
        throw std::runtime_error("String is not valid length ...");
    }

    size_t cnt = str.length() / 2;

    for (size_t i = 0; cnt > i; ++i) {
        uint32_t s = 0;
        std::stringstream ss;
        ss << std::hex << str.substr(i * 2, 2);
        ss >> s;

        output.push_back(static_cast<unsigned char>(s));
    }

    return output;
}

void Zeta::look(){
    pr2_controllers_msgs::PointHeadGoal g;
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_footprint";
    point.header.stamp = ros::Time::now();
    point.point.x = 1.5;
    point.point.y = 0.0;
    point.point.z = 1.6;
    g.target = point;
    g.pointing_frame = "high_def_frame";
    g.pointing_axis.x = 1;
    g.pointing_axis.y = 0;
    g.pointing_axis.z = 0;
    g.min_duration = ros::Duration(5.0);
    g.max_velocity = 1.0;
    headact_.sendGoal(g);
    headact_.waitForResult();
}