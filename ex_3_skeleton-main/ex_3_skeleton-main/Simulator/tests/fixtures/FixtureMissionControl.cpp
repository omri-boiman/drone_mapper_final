// A minimal, REAL (dlopen-able) MissionControl .so used only to drive the
// multi-plugin comparative-mode tests (test_multiplugin.cpp) with
// deterministic, controllable totals -- see ex3-test-plan.md section 3.
//
// Deliberately does NOT drive the injected algorithm/drone at all: it never
// calls mapping_algorithm.nextStep(), never moves, never scans. It just
// reports "Completed" after exactly FIXTURE_STEPS (a compile-time constant,
// baked in per .so via -DFIXTURE_STEPS=<n>) steps and saves the (untouched)
// output map. That means every instance of this fixture produces the SAME
// mission_score (whatever MapsComparison gives an all-Empty predicted map
// against the real hidden map) but a CONTROLLABLE, DIFFERENT total_steps --
// exactly what's needed to test writeComparativeReport's "agreeing managers"
// grouping (grouped by the (total_score, total_steps) pair) without needing
// the fixture to fake map content.
//
// Built multiple times from this one source file with different
// -DFIXTURE_STEPS / -DFIXTURE_CLASS_NAME combinations (see
// Simulator/CMakeLists.txt) -- each resulting .so is fully independent, no
// symbol collisions between them (dlopen with RTLD_LOCAL isolates each
// plugin's symbols from every other).

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>

#include <cstddef>
#include <filesystem>
#include <utility>

#ifndef FIXTURE_STEPS
#define FIXTURE_STEPS 0
#endif
#ifndef FIXTURE_CLASS_NAME
#define FIXTURE_CLASS_NAME FixtureMissionControlDefault
#endif

using namespace common;

class FIXTURE_CLASS_NAME final : public IMissionControl {
public:
    explicit FIXTURE_CLASS_NAME(MissionControlDependencies deps)
        : output_map_(deps.output_map), output_map_file_(std::move(deps.output_map_file)) {}

    types::MissionRunResult runMission() override {
        output_map_.save(output_map_file_);
        return {types::MissionRunStatus::Completed, static_cast<std::size_t>(FIXTURE_STEPS), {}};
    }

private:
    IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
};

REGISTER_MISSION_CONTROL(FIXTURE_CLASS_NAME);
