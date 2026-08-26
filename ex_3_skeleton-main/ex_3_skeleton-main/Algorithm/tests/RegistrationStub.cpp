// MappingAlgorithmImpl.cpp calls REGISTER_MAPPING_ALGORITHM(...) at global
// scope, which needs MappingAlgorithmRegistration's constructor to be defined
// somewhere. In the real build that definition lives in the Simulator
// project; for a standalone Algorithm test binary we only need it to LINK,
// not actually register anything, so a no-op body is enough.
#include <Common/MappingAlgorithmRegistration.h>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory) {}

} // namespace common
