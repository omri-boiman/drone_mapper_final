#pragma once

#include <string>
#include <vector>
#include <filesystem>

namespace drone {

// Collects recoverable input-file errors and writes them to input_errors.txt.
// The file is created only if at least one error was logged.
class ErrorLogger {
public:
    // Log a recoverable error message (does not write to disk yet)
    void Log(const std::string& message);

    // Returns true if any errors have been logged
    bool HasErrors() const;

    // Writes all logged errors to <dir>/input_errors.txt.
    // Does nothing if no errors were logged.
    // Returns true on success, false if the file could not be written.
    bool Flush(const std::filesystem::path& dir) const;

private:
    std::vector<std::string> m_errors;
};

} // namespace drone
