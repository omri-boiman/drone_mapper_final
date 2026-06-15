#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace drone_mapper {

class ErrorLogger {
public:
    explicit ErrorLogger(const std::filesystem::path& path);
    void log(const std::string& message);
    [[nodiscard]] bool hasErrors() const noexcept;

private:
    std::filesystem::path path_;
    std::ofstream         stream_;
    bool                  has_errors_ = false;
};

} // namespace drone_mapper
