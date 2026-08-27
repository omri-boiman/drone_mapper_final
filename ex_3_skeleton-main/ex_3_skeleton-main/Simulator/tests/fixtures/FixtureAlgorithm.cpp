// A minimal, REAL (dlopen-able) Algorithm .so used only to drive the
// multi-plugin competition-mode tests (test_multiplugin.cpp) with
// deterministic, controllable totals -- see ex3-test-plan.md section 3.
//
// Returns a pure no-op step ({nullopt movement, nullopt scan, Working}) --
// confirmed safe by inspection of DroneControlImpl::step(), which does
// nothing when both are nullopt -- exactly FIXTURE_STEPS times, then
// Finished. It never scans, so the output map stays untouched and every
// instance of this fixture produces the SAME mission_score, but a
// CONTROLLABLE, DIFFERENT total_steps -- enough to test
// writeCompetitiveReport's steps-ascending tie-break sort without depending
// on real navigation/scan geometry.
//
// Built multiple times from this one source file with different
// -DFIXTURE_STEPS / -DFIXTURE_CLASS_NAME combinations (see
// Simulator/CMakeLists.txt).

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MappingAlgorithmRegistration.h>

#ifndef FIXTURE_STEPS
#define FIXTURE_STEPS 0
#endif
#ifndef FIXTURE_CLASS_NAME
#define FIXTURE_CLASS_NAME FixtureAlgorithmDefault
#endif

using namespace common;

class FIXTURE_CLASS_NAME final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;

    types::MappingStepCommand nextStep(const types::DroneState&,
                                       const types::LidarScanResult*) override {
        if (calls_++ >= FIXTURE_STEPS) {
            return {std::nullopt, std::nullopt, types::AlgorithmStatus::Finished};
        }
        return {std::nullopt, std::nullopt, types::AlgorithmStatus::Working};
    }

private:
    int calls_ = 0;
};

REGISTER_MAPPING_ALGORITHM(FIXTURE_CLASS_NAME);
