#include "RunModes.h"

#include <Simulator/ScoreReportWriter.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/YamlConfigParser.h>

#include "plugin/PluginLoader.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <system_error>
#include <thread>

namespace simulator {

std::string timestampTag() {
    const auto now = std::chrono::system_clock::now();
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
    return std::to_string(us);
}

std::vector<std::filesystem::path> soFilesIn(const std::filesystem::path& folder) {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        if (entry.path().extension() == ".so") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end()); // deterministic run order
    return files;
}

std::optional<std::filesystem::path> makeOutputDir(const std::filesystem::path& base,
                                                    const std::string& prefix) {
    const auto dir = base / (prefix + timestampTag());
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::fprintf(stderr, "could not create output directory %s: %s\n",
                     dir.c_str(), ec.message().c_str());
        return std::nullopt;
    }
    return dir;
}

PluginTotals totalsFor(const std::string& so_name,
                       const types::SimulationManagerReport& report) {
    double score_sum = 0.0;
    std::size_t steps_sum = 0;
    for (const auto& run : report.runs) {
        score_sum += run.mission_score;
        for (const auto& mr : run.mission_results) steps_sum += mr.steps;
    }
    return {so_name, score_sum, steps_sum};
}

std::vector<std::pair<std::size_t, std::size_t>> partitionWork(
    std::size_t total_items, std::optional<unsigned> num_threads_requested) {
    if (total_items == 0) return {};

    std::size_t requested_extra =
        (num_threads_requested && *num_threads_requested >= 2) ? *num_threads_requested : 0;
    const std::size_t max_extra_by_work = total_items - 1;
    std::size_t extra_threads = std::min(requested_extra, max_extra_by_work);
    if (extra_threads == 1) extra_threads = 0;

    const std::size_t total_workers = 1 + extra_threads;
    std::vector<std::pair<std::size_t, std::size_t>> chunks;
    chunks.reserve(total_workers);
    const std::size_t base = total_items / total_workers;
    const std::size_t rem  = total_items % total_workers;
    std::size_t start = 0;
    for (std::size_t w = 0; w < total_workers; ++w) {
        const std::size_t size = base + (w < rem ? 1 : 0);
        chunks.emplace_back(start, start + size);
        start += size;
    }
    return chunks;
}

namespace {

// Runs `factory_for(i)` against `fixed_factory` for every loaded plugin,
// across `chunks` (chunk 0 inline on the caller's thread, the rest on spawned
// workers), writing each plugin's legacy YAML into `output_dir` (shared, safe
// -- distinct filename per plugin) and its output maps/error log into
// `output_dir/<so filename>/` (a PER-PLUGIN subdirectory -- this is what
// makes the parallel case safe: SimulationManager::run() unconditionally
// writes to `<output_path>/output_results/error.log`, so without a private
// subdirectory per plugin, concurrent threads would race on the same
// error.log file. Giving each plugin its own subdirectory sidesteps that
// entirely, no locking needed, and still satisfies the spec's own wording --
// "Error log(s) files" is explicitly plural).
template <typename FactoryForFn, typename BuildRunFactoryFn>
void runWorkItems(const std::vector<std::filesystem::path>& so_files,
                  FactoryForFn factory_for, // (size_t i) -> optional<Loaded*Plugin>&
                  BuildRunFactoryFn build_run_factory, // (varying_plugin) -> unique_ptr<SimulationRunFactoryImpl>
                  const YamlConfigParser::CompositionWithPaths& paths,
                  const std::filesystem::path& output_dir,
                  const std::vector<std::pair<std::size_t, std::size_t>>& chunks,
                  std::vector<PluginTotals>& totals_out,
                  std::vector<std::string>& errors_out) {
    std::vector<PluginTotals> per_index_totals(so_files.size());
    std::vector<bool> succeeded(so_files.size(), false);

    auto processRange = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            auto& varying_plugin = factory_for(i);
            if (!varying_plugin) {
                std::fprintf(stderr, "%s: failed to load\n", so_files[i].filename().c_str());
                continue;
            }

            const auto plugin_output_dir = output_dir / so_files[i].filename().string();

            auto run_factory = build_run_factory(*varying_plugin);
            SimulationManager manager(std::move(run_factory));

            try {
                auto report = manager.run(paths.data, plugin_output_dir);
                ScoreReportWriter::write(report, paths, output_dir,
                    "simulation_output_" + so_files[i].filename().string() + ".yaml");
                per_index_totals[i] = totalsFor(so_files[i].filename().string(), report);
                succeeded[i] = true;
            } catch (const std::exception& e) {
                std::fprintf(stderr, "%s: %s\n", so_files[i].filename().c_str(), e.what());
            }
            // Note: `varying_plugin` is a reference into the caller's plugin
            // vector, not a local -- it stays loaded until that whole vector
            // is destroyed (see runComparative/runCompetition), by design:
            // all plugins are loaded up front for the threaded phase, per
            // the spec's own suggestion ("load all the .so files ahead").
        }
    };

    std::vector<std::thread> workers;
    for (std::size_t w = 1; w < chunks.size(); ++w) {
        workers.emplace_back(processRange, chunks[w].first, chunks[w].second);
    }
    if (!chunks.empty()) processRange(chunks[0].first, chunks[0].second);
    for (auto& t : workers) t.join();

    for (std::size_t i = 0; i < so_files.size(); ++i) {
        if (succeeded[i]) totals_out.push_back(per_index_totals[i]);
        else errors_out.push_back(so_files[i].filename().string());
    }
}

} // namespace

int runComparative(const ParsedArgs& args) {
    std::string error;
    auto algorithm_plugin = loadAlgorithmPlugin(args.algorithm, error);
    if (!algorithm_plugin) {
        std::fprintf(stderr, "failed to load algorithm: %s\n", error.c_str());
        return 1;
    }

    const auto output_dir = makeOutputDir(args.mission_control_folder, "comparative_results_");
    if (!output_dir) return 1;

    const auto paths = YamlConfigParser::parseCompositionWithPaths(args.simulation);
    const auto mc_files = soFilesIn(args.mission_control_folder);

    // Phase 1: load every mission-control .so serially, on this thread, before
    // any worker is spawned (dlopen + PluginRegistrar-drain is not meant to
    // run concurrently -- see project notes). Phase 2 workers only ever
    // invoke an already-captured factory.
    std::vector<std::optional<LoadedMissionControlPlugin>> mc_plugins(mc_files.size());
    for (std::size_t i = 0; i < mc_files.size(); ++i) {
        std::string load_error;
        mc_plugins[i] = loadMissionControlPlugin(mc_files[i], load_error);
    }

    const auto chunks = partitionWork(mc_files.size(), args.num_threads);

    std::vector<PluginTotals> totals;
    std::vector<std::string> errors;
    runWorkItems(
        mc_files,
        [&](std::size_t i) -> std::optional<LoadedMissionControlPlugin>& { return mc_plugins[i]; },
        [&](const LoadedMissionControlPlugin& mc_plugin) {
            return std::make_unique<SimulationRunFactoryImpl>(
                algorithm_plugin->factory, mc_plugin.factory, args.verbose);
        },
        paths, *output_dir, chunks, totals, errors);

    writeComparativeReport(args.simulation, args.mission_control_folder,
                           totals, errors, *output_dir);

    std::printf("Comparative run: %zu succeeded, %zu failed. Results in \"%s\"\n",
                totals.size(), errors.size(), output_dir->c_str());
    return 0;
    // mc_plugins go out of scope here -> dlclose for every loaded .so, only
    // after every worker thread has joined and every instance built from
    // them is gone.
}

int runCompetition(const ParsedArgs& args) {
    std::string error;
    auto mission_control_plugin = loadMissionControlPlugin(args.mission_control, error);
    if (!mission_control_plugin) {
        std::fprintf(stderr, "failed to load mission control: %s\n", error.c_str());
        return 1;
    }

    const auto output_dir = makeOutputDir(args.algorithms_folder, "competition_");
    if (!output_dir) return 1;

    const auto paths = YamlConfigParser::parseCompositionWithPaths(args.simulation);
    const auto algo_files = soFilesIn(args.algorithms_folder);

    std::vector<std::optional<LoadedAlgorithmPlugin>> algo_plugins(algo_files.size());
    for (std::size_t i = 0; i < algo_files.size(); ++i) {
        std::string load_error;
        algo_plugins[i] = loadAlgorithmPlugin(algo_files[i], load_error);
    }

    const auto chunks = partitionWork(algo_files.size(), args.num_threads);

    std::vector<PluginTotals> totals;
    std::vector<std::string> errors;
    runWorkItems(
        algo_files,
        [&](std::size_t i) -> std::optional<LoadedAlgorithmPlugin>& { return algo_plugins[i]; },
        [&](const LoadedAlgorithmPlugin& algo_plugin) {
            return std::make_unique<SimulationRunFactoryImpl>(
                algo_plugin.factory, mission_control_plugin->factory, args.verbose);
        },
        paths, *output_dir, chunks, totals, errors);

    writeCompetitiveReport(args.simulation, args.mission_control,
                           totals, errors, *output_dir);

    std::printf("Competitive run: %zu succeeded, %zu failed. Results in \"%s\"\n",
                totals.size(), errors.size(), output_dir->c_str());
    return 0;
    // mission_control_plugin destructs here, after every worker thread has
    // joined and every mission-control instance built from it is gone.
}

} // namespace simulator
