// Gravity particle simulation example for the KDTree library.
//
// DISCLAIMER: This entire usage example (Simulation.h/.cpp, main.cpp) was written with the help of
// Claude Sonnet 5 (Anthropic's AI coding assistant). This project is not meant to be used in production but
// merely to be regarded as a proof of concept and an usage example.
//
// Particles interact under Newtonian gravity. To avoid an O(n^2) pairwise force sum,
// each simulation step builds a KDTree over the current particle positions and reads
// out its leaf nodes: particles sharing a leaf are close together and are summed
// directly, while every other leaf is collapsed into a single point mass (its combined
// mass, located at its center of mass) for a cheap approximate contribution.
#include "Simulation.h"

#include "KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "KDTree/util/UtilityContainer.h"

#include <CLI/CLI.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

    struct Options {
        gravity_demo::SimulationConfig simulation{};
        std::size_t steps = 200;
        std::string outputDir = "output";
        std::size_t outputInterval = 5;
    };

    [[nodiscard]] std::string toUpper(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
            return static_cast<char>(std::toupper(c));
        });
        return value;
    }

    [[nodiscard]] kdtree::PlaneSelectionAlgorithm::Algorithm parseAlgorithm(const std::string &value) {
        const std::string upper = toUpper(value);
        if (upper == "LOG") return kdtree::PlaneSelectionAlgorithm::Algorithm::LOG;
        if (upper == "LOGSQUARED") return kdtree::PlaneSelectionAlgorithm::Algorithm::LOGSQUARED;
        if (upper == "QUADRATIC") return kdtree::PlaneSelectionAlgorithm::Algorithm::QUADRATIC;
        if (upper == "NOTREE") return kdtree::PlaneSelectionAlgorithm::Algorithm::NOTREE;
        throw std::invalid_argument("Unknown algorithm '" + value + "'. Use LOG, LOGSQUARED, QUADRATIC or NOTREE.");
    }

    void writeFrame(const std::filesystem::path &outputDir, const std::size_t step, const gravity_demo::Simulation &simulation) {
        std::filesystem::create_directories(outputDir);
        std::ostringstream fileName;
        fileName << "frame_" << std::setw(6) << std::setfill('0') << step << ".csv";
        std::ofstream file(outputDir / fileName.str());
        file << "index,x,y,z,vx,vy,vz,mass\n";

        const auto &positions = simulation.positions();
        const auto &velocities = simulation.velocities();
        const auto &masses = simulation.masses();
        for (std::size_t i = 0; i < positions.size(); ++i) {
            file << i << ','
                 << positions[i][0] << ',' << positions[i][1] << ',' << positions[i][2] << ','
                 << velocities[i][0] << ',' << velocities[i][1] << ',' << velocities[i][2] << ','
                 << masses[i] << '\n';
        }
    }

    /**
     * Runs a KDTree-approximated simulation and a brute-force one side by side from
     * identical initial conditions (same seed) and reports, per step, how far apart their
     * particle positions and kinetic energy have drifted. Any divergence is entirely down to
     * the KDTree leaf-cluster force approximation, though it compounds over time since N-body
     * gravity is chaotic: even a tiny per-step force error grows into large trajectory
     * differences after enough steps.
     */
    int runAccuracyComparison(const Options &options) {
        using namespace kdtree::util;

        gravity_demo::SimulationConfig kdTreeConfig = options.simulation;
        kdTreeConfig.useKdTree = true;
        gravity_demo::SimulationConfig bruteForceConfig = options.simulation;
        bruteForceConfig.useKdTree = false;

        gravity_demo::Simulation kdTreeSimulation(kdTreeConfig);
        gravity_demo::Simulation bruteForceSimulation(bruteForceConfig);

        std::cout << "Comparing accuracy: " << options.simulation.particleCount << " particles, "
                  << options.steps << " steps, dt=" << options.simulation.timeStep
                  << " (KDTree leaf-cluster approximation vs. brute-force O(n^2), identical initial conditions)\n";

        std::cout << "step,rms_position_error,max_position_error,kinetic_energy_kdtree,kinetic_energy_brute_force,relative_energy_error\n";
        double peakRmsError = 0.0;
        for (std::size_t step = 1; step <= options.steps; ++step) {
            kdTreeSimulation.step();
            bruteForceSimulation.step();

            const auto &kdPositions = kdTreeSimulation.positions();
            const auto &brutePositions = bruteForceSimulation.positions();

            double sumSquaredError = 0.0;
            double maxSquaredError = 0.0;
            for (std::size_t i = 0; i < kdPositions.size(); ++i) {
                const auto delta = kdPositions[i] - brutePositions[i];
                const double squaredError = dot(delta, delta);
                sumSquaredError += squaredError;
                maxSquaredError = std::max(maxSquaredError, squaredError);
            }
            const double rmsError = std::sqrt(sumSquaredError / static_cast<double>(kdPositions.size()));
            const double maxError = std::sqrt(maxSquaredError);
            peakRmsError = std::max(peakRmsError, rmsError);

            const double kdEnergy = kdTreeSimulation.totalKineticEnergy();
            const double bruteEnergy = bruteForceSimulation.totalKineticEnergy();
            const double relativeEnergyError = bruteEnergy != 0.0 ? std::abs(kdEnergy - bruteEnergy) / std::abs(bruteEnergy) : 0.0;

            std::cout << step << ',' << rmsError << ',' << maxError << ',' << kdEnergy << ',' << bruteEnergy << ','
                      << relativeEnergyError << '\n';
        }

        std::cout << "Accuracy comparison finished. Peak RMS position error over the run: " << peakRmsError << ".\n";
        return 0;
    }

}// namespace

int main(int argc, char **argv) {
    CLI::App app{"Gravity particle simulation using the KDTree library for leaf-cluster force approximation."};
    argv = app.ensure_utf8(argv);
    app.option_defaults()->always_capture_default();

    Options options{};
    std::string algorithmName = "LOG";
    bool debug{false};

    app.add_option("--particles", options.simulation.particleCount, "Number of particles")->check(CLI::PositiveNumber);
    app.add_option("--steps", options.steps, "Number of simulation steps to run");
    app.add_option("--dt", options.simulation.timeStep, "Time step size");
    app.add_option("--extent", options.simulation.halfExtent, "Half-extent of the initial spherical particle cloud");
    app.add_option("--mass", options.simulation.particleMass, "Mass assigned to every particle");
    app.add_option("--gravity", options.simulation.gravitationalConstant, "Gravitational constant G");
    app.add_option("--softening", options.simulation.softening, "Softening length to avoid singularities at close range");
    app.add_option("--spin", options.simulation.initialSpin, "Initial tangential velocity factor, 0 disables");
    app.add_option("--seed", options.simulation.seed, "Random seed for the initial particle cloud");
    app.add_option("--algorithm", algorithmName, "KDTree plane selection algorithm (LOG, LOGSQUARED, QUADRATIC, NOTREE)")
            ->transform(CLI::IsMember({"LOG", "LOGSQUARED", "QUADRATIC", "NOTREE"}, CLI::ignore_case));
    app.add_flag("--adaptive-rebuild,!--no-adaptive-rebuild", options.simulation.adaptiveRebuild,
                 "Only rebuild the KDTree when a particle has left its leaf's bounding box "
                 "(KDTree::rebuildTreeIfNeeded()) instead of unconditionally every step");
    auto *kdTreeFlag = app.add_flag("--kd-tree,!--brute-force", options.simulation.useKdTree,
                                    "Use the KDTree leaf-cluster approximation (default) to compute forces, "
                                    "or a brute-force O(n^2) pairwise sum with --brute-force, to compare performance");
    app.add_option("--output-dir", options.outputDir, "Directory CSV frames are written to");
    app.add_option("--output-interval", options.outputInterval, "Steps between CSV frame dumps, 0 disables output");
    app.add_flag("--debug", debug, "Enables debug mode");
    bool compareAccuracy{false};
    app.add_flag("--compare-accuracy", compareAccuracy,
                 "Instead of a normal run, advance a KDTree-approximated simulation and a brute-force "
                 "one side by side from identical initial conditions and report their per-step position/"
                 "energy divergence, to measure the accuracy cost of the KDTree approximation. "
                 "Ignores --kd-tree/--brute-force and CSV frame output.")
            ->excludes(kdTreeFlag);

    CLI11_PARSE(app, argc, argv);
    options.simulation.algorithm = parseAlgorithm(algorithmName);

    try {
        if (compareAccuracy) {
            return runAccuracyComparison(options);
        }

        std::cout << "Gravity simulation: " << options.simulation.particleCount << " particles, "
                  << options.steps << " steps, dt=" << options.simulation.timeStep << ", force mode="
                  << (options.simulation.useKdTree ? "KDTree leaf-cluster approximation" : "brute-force O(n^2)") << '\n';
        if (options.outputInterval > 0) {
            std::cout << "Writing CSV frames every " << options.outputInterval << " step(s) to '"
                      << options.outputDir << "'\n";
        }

        gravity_demo::Simulation simulation(options.simulation);

        if (options.outputInterval > 0) {
            writeFrame(options.outputDir, 0, simulation);
        }

        std::cout << "step,elapsed_ms,clusters,rebuilt,kinetic_energy\n";
        double totalElapsedMs = 0.0;
        for (std::size_t step = 1; step <= options.steps; ++step) {
            const auto start = std::chrono::steady_clock::now();
            simulation.step();
            const auto elapsed = std::chrono::steady_clock::now() - start;
            const double elapsedMs = std::chrono::duration<double, std::milli>(elapsed).count();
            totalElapsedMs += elapsedMs;

            std::cout << step << ',' << elapsedMs << ',' << simulation.lastClusterCount() << ','
                      << (simulation.lastRebuildOccurred() ? "rebuild" : "noop") << ','
                      << simulation.totalKineticEnergy() << '\n';

            if (options.outputInterval > 0 && step % options.outputInterval == 0) {
                writeFrame(options.outputDir, step, simulation);
            }
        }

        std::cout << "Simulation finished. Total simulation time: " << totalElapsedMs << " ms across "
                  << options.steps << " steps (avg " << (totalElapsedMs / static_cast<double>(options.steps))
                  << " ms/step).\n";
        if (debug) {
            std::cout << "Final kd-tree:" << std::endl;
            simulation.printKdTree();
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
