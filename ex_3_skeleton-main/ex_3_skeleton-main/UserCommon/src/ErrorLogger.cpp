#include <UserCommon/ErrorLogger.h>

#include <chrono>
#include <iomanip>
#include <sstream>

namespace user_common_211781141_325049575 {

ErrorLogger::ErrorLogger(const std::filesystem::path& path) : path_(path) {
    std::filesystem::create_directories(path_.parent_path());
    stream_.open(path_, std::ios::out | std::ios::app);
}

void ErrorLogger::log(const std::string& message) {
    if (!stream_.is_open()) return;

    const auto now = std::chrono::system_clock::now();
    const auto tt  = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ts;
    ts << std::put_time(std::gmtime(&tt), "[%Y-%m-%dT%H:%M:%SZ] ");

    stream_ << ts.str() << message << '\n';
    stream_.flush();
    has_errors_ = true;
}

bool ErrorLogger::hasErrors() const noexcept {
    return has_errors_;
}

} // namespace user_common_211781141_325049575
