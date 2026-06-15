#include "io/ErrorLogger.h"

#include <fstream>
#include <iostream>

namespace drone {

void ErrorLogger::Log(const std::string& message)
{
    m_errors.push_back(message);
}

bool ErrorLogger::HasErrors() const
{
    return !m_errors.empty();
}

bool ErrorLogger::Flush(const std::filesystem::path& dir) const
{
    if (m_errors.empty()) {
        return true;
    }

    const std::filesystem::path outPath = dir / "input_errors.txt";
    std::ofstream file(outPath);
    if (!file.is_open()) {
        std::cerr << "Warning: could not write input_errors.txt to "
                  << outPath << "\n";
        return false;
    }

    for (const auto& msg : m_errors) {
        file << msg << "\n";
    }
    return true;
}

} // namespace drone
