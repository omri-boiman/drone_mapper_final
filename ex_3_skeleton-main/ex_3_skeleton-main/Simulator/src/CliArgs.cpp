#include "CliArgs.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <system_error>

namespace simulator {

namespace {

void printUsage(const char* program_name) {
    std::fprintf(stderr,
        "usage:\n"
        "  %s -comparative simulation=<file> mission_control_folder=<folder> "
        "algorithm=<file> [num_threads=<num>] [-verbose]\n"
        "  %s -competition simulation=<file> mission_control=<file> "
        "algorithms_folder=<folder> [num_threads=<num>] [-verbose]\n",
        program_name, program_name);
}

void printErrors(const std::vector<std::string>& errors) {
    for (const auto& e : errors) {
        std::fprintf(stderr, "error: %s\n", e.c_str());
    }
}

bool fileExistsAndOpenable(const std::filesystem::path& p) {
    std::error_code ec;
    if (!std::filesystem::exists(p, ec) || ec) return false;
    if (!std::filesystem::is_regular_file(p, ec) || ec) return false;
    std::ifstream f(p);
    return f.good();
}

bool folderHasAtLeastOneSo(const std::filesystem::path& folder, std::string& detail) {
    std::error_code ec;
    if (!std::filesystem::exists(folder, ec) || ec || !std::filesystem::is_directory(folder, ec) || ec) {
        detail = "does not exist or is not a directory";
        return false;
    }
    std::filesystem::directory_iterator it(folder, ec);
    if (ec) {
        detail = "cannot be traversed: " + ec.message();
        return false;
    }
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        if (entry.path().extension() == ".so") return true;
    }
    detail = "contains no .so files";
    return false;
}

} // namespace

std::optional<ParsedArgs> parseAndValidateArgs(
    const std::vector<std::string>& args, const char* program_name) {

    bool comparative_flag = false;
    bool competition_flag = false;
    bool verbose_flag = false;
    std::map<std::string, std::string> kv;
    std::vector<std::string> unsupported;

    for (const auto& a : args) {
        if (a == "-comparative") { comparative_flag = true; continue; }
        if (a == "-competition") { competition_flag = true; continue; }
        if (a == "-verbose")     { verbose_flag = true; continue; }

        const auto eq = a.find('=');
        if (eq != std::string::npos && eq > 0) {
            kv[a.substr(0, eq)] = a.substr(eq + 1);
        } else {
            unsupported.push_back(a);
        }
    }

    std::vector<std::string> errors;

    if (comparative_flag && competition_flag) {
        errors.push_back("cannot specify both -comparative and -competition");
    } else if (!comparative_flag && !competition_flag) {
        errors.push_back("missing required mode flag: -comparative or -competition");
    }

    if (!errors.empty()) {
        printUsage(program_name);
        printErrors(errors);
        return std::nullopt;
    }

    const Mode mode = comparative_flag ? Mode::Comparative : Mode::Competition;

    const std::set<std::string> required = (mode == Mode::Comparative)
        ? std::set<std::string>{"simulation", "mission_control_folder", "algorithm"}
        : std::set<std::string>{"simulation", "mission_control", "algorithms_folder"};
    const std::set<std::string> optional_keys = {"num_threads"};

    for (const auto& [key, value] : kv) {
        if (!required.count(key) && !optional_keys.count(key)) {
            unsupported.push_back(key + "=" + value);
        }
    }
    for (const auto& key : required) {
        if (!kv.count(key)) {
            errors.push_back("missing required argument: " + key);
        }
    }
    if (!unsupported.empty()) {
        std::string joined;
        for (const auto& u : unsupported) {
            if (!joined.empty()) joined += ", ";
            joined += u;
        }
        errors.push_back("unsupported argument(s): " + joined);
    }

    std::optional<unsigned> num_threads;
    if (auto it = kv.find("num_threads"); it != kv.end()) {
        try {
            std::size_t consumed = 0;
            const long parsed = std::stol(it->second, &consumed);
            if (parsed < 0 || consumed != it->second.size()) throw std::invalid_argument("");
            num_threads = static_cast<unsigned>(parsed);
        } catch (const std::exception&) {
            errors.push_back("num_threads must be a non-negative integer, got: " + it->second);
        }
    }

    if (!errors.empty()) {
        printUsage(program_name);
        printErrors(errors);
        return std::nullopt;
    }

    // Existence checks -- only reached once the argument SET itself is well-formed.
    ParsedArgs parsed;
    parsed.mode = mode;
    parsed.simulation = kv.at("simulation");
    parsed.num_threads = num_threads;
    parsed.verbose = verbose_flag;

    if (!fileExistsAndOpenable(parsed.simulation)) {
        errors.push_back("simulation file not found or cannot be opened: " + parsed.simulation.string());
    }

    if (mode == Mode::Comparative) {
        parsed.mission_control_folder = kv.at("mission_control_folder");
        parsed.algorithm = kv.at("algorithm");

        if (!fileExistsAndOpenable(parsed.algorithm)) {
            errors.push_back("algorithm file not found or cannot be opened: " + parsed.algorithm.string());
        }
        std::string detail;
        if (!folderHasAtLeastOneSo(parsed.mission_control_folder, detail)) {
            errors.push_back("mission_control_folder " + parsed.mission_control_folder.string() +
                             " " + detail);
        }
    } else {
        parsed.mission_control = kv.at("mission_control");
        parsed.algorithms_folder = kv.at("algorithms_folder");

        if (!fileExistsAndOpenable(parsed.mission_control)) {
            errors.push_back("mission_control file not found or cannot be opened: " + parsed.mission_control.string());
        }
        std::string detail;
        if (!folderHasAtLeastOneSo(parsed.algorithms_folder, detail)) {
            errors.push_back("algorithms_folder " + parsed.algorithms_folder.string() +
                             " " + detail);
        }
    }

    if (!errors.empty()) {
        printUsage(program_name);
        printErrors(errors);
        return std::nullopt;
    }

    return parsed;
}

} // namespace simulator
