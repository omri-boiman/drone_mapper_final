#pragma once

// Extracted out of main.cpp (which originally kept all of this in its own
// anonymous namespace) so it can be unit/integration-tested directly. main.cpp
// is now a thin wrapper: parse argv via CliArgs, dispatch to runComparative/
// runCompetition below. Behavior is unchanged from the original main.cpp --
// this is a pure extraction, not a rewrite. See test_partition_work.cpp and
// test_multiplugin.cpp for the tests this extraction exists to enable.

#include "AggregateReport.h"
#include "CliArgs.h"

#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace simulator {

// A new numeric tag per call (microseconds since epoch) -- used to build
// collision-avoiding output-directory names. Not guaranteed monotonic across
// processes, only distinct within the lifetime of one.
std::string timestampTag();

// Every ".so" file directly inside `folder`, sorted for deterministic run
// order. Throws std::filesystem::filesystem_error if `folder` doesn't exist
// or can't be traversed -- callers (CliArgs::parseAndValidateArgs) are
// expected to have already validated that before this is called for real.
std::vector<std::filesystem::path> soFilesIn(const std::filesystem::path& folder);

// Creates `base/(prefix + timestampTag())`. Returns nullopt (after printing a
// "could not create output directory" message to stderr) if creation fails --
// per spec, both comparative and competitive modes must report this to
// screen rather than crash.
std::optional<std::filesystem::path> makeOutputDir(const std::filesystem::path& base,
                                                    const std::string& prefix);

// Splits [0, total_items) into contiguous, disjoint chunks: chunk 0 is
// processed by the caller's own thread, every remaining chunk gets one
// std::thread of its own. extra_threads = min(requested, total_items - 1),
// collapsed to 0 whenever that would land on exactly 1 extra thread (spec:
// "the total number of threads will never be 2"). Returns an empty vector
// for total_items == 0.
std::vector<std::pair<std::size_t, std::size_t>> partitionWork(
    std::size_t total_items, std::optional<unsigned> num_threads_requested);

// Sums mission_score/steps across every run in one plugin's report, for the
// aggregate comparative_report.yaml / competitive_report.yaml totals.
PluginTotals totalsFor(const std::string& so_name,
                       const types::SimulationManagerReport& report);

// Real entry points: dlopen every varying-side .so up front (serially, main
// thread only -- see PluginRegistrar.h for why), partition the work across
// `args.num_threads`, run each plugin's full composition, write its legacy
// per-plugin YAML plus the shared aggregate report, and return a process
// exit code (0 on completion -- per-plugin failures are collected into the
// aggregate report's `errors:` list, not treated as a fatal error).
int runComparative(const ParsedArgs& args);
int runCompetition(const ParsedArgs& args);

} // namespace simulator
