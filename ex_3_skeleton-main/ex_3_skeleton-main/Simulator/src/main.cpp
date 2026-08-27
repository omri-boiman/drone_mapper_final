#include "CliArgs.h"
#include "RunModes.h"

#include <cstdio>
#include <string>
#include <vector>

// main.cpp is intentionally thin: parse+validate argv, then dispatch to
// RunModes.h's runComparative/runCompetition. The actual orchestration logic
// (plugin loading, thread partitioning, per-plugin run + aggregate report)
// used to live in an anonymous namespace here; it was extracted into
// RunModes.h/.cpp so it can be exercised directly by tests (see
// Simulator/tests/test_partition_work.cpp and test_multiplugin.cpp) --
// main.cpp itself is excluded from every test target (nothing in Simulator's
// own test code calls the REGISTER_* macros, so it's not linked in), so
// logic left here would otherwise be completely untestable.

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);

    const auto parsed = simulator::parseAndValidateArgs(args, argv[0]);
    if (!parsed) return 1;

    // Top-level guard: CLI validation only checks that files exist/open, not
    // that their *contents* are well-formed (e.g. a malformed simulation.yaml
    // missing required keys). Catch here so a bad composition file reports an
    // error instead of an uncaught-exception crash, per "the Simulator shall
    // not crash" (crashes inside MissionControl/Algorithm are explicitly
    // exempted by the spec, but this exception originates in the Simulator
    // itself, before any plugin code runs).
    try {
        return (parsed->mode == simulator::Mode::Comparative)
            ? simulator::runComparative(*parsed)
            : simulator::runCompetition(*parsed);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
