#pragma once

#include <string>

namespace chassis
{
namespace bxi_timing
{

// Call before the first BXI transport is constructed. Only CAN6 is configured;
// accepts 250000, 500000 or 1000000 bit/s, otherwise throws invalid_argument.
void configure_can6_bitrate(unsigned bitrate);

// Used around bxi_pci_init(). A nonempty result must reject hardware startup.
void before_init();
std::string initialization_error();

}  // namespace bxi_timing
}  // namespace chassis
