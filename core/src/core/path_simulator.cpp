//
// Copyright 2017-2023 Valve Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "path_simulator.h"

#include "eq_effect.h"
#include "math_functions.h"
#include "sampling.h"
#include "sh.h"
#include "profiler.h"
#include "propagation_medium.h"

namespace ipl {

// --------------------------------------------------------------------------------------------------------------------
// PathSimulator
// --------------------------------------------------------------------------------------------------------------------

bool PathSimulator::sEnablePathsFromAllSourceProbes = false;

PathSimulator::PathSimulator(const ProbeBatch& probes,
                             int numSamples,
                             bool asymmetricVisRange,
                             const Vector3f& down)
    : mVisTester(numSamples, asymmetricVisRange, down)
    , mPathFinder(probes, 1)
{
    mProbePath.nodes.reserve(16);
    mRoutePoints.reserve(18);
    for (auto& path : mResolvedPaths)
        path.bends.reserve(16);
}

bool PathSimulator::isPathOccluded(const SoundPath& path,
                                   const IScene& scene,
                                   const ProbeBatch& probes,
                                   float radius,
                                   float threshold,
                                   int start,
                                   int end,
                                   bool enableValidation,
                                   ValidationRayVisualizationCallback validationRayVisualization,
                                   void* userData,
                                   bool visualizeValidSegments) const
{
    PROFILE_FUNCTION();

    if (enableValidation || validationRayVisualization)
    {
        BakedDataIdentifier identifier;
        identifier.variation = BakedDataVariation::Dynamic;
        identifier.type = BakedDataType::Pathing;

        const auto& bakedPathData = static_cast<const BakedPathData&>(probes[identifier]);

        auto current = end;
        auto prev = (path.direct) ? start : path.lastProbe;

        while (current != start)
        {
            bool probeVisible = enableValidation ? mVisTester.areProbesVisible(scene, probes, current, prev, radius, threshold) : true;
            if (!probeVisible)
            {
                if (validationRayVisualization)
                {
                    validationRayVisualization(probes[prev].influence.center,
                                               probes[current].influence.center, true, userData);
                }

                return true;
            }
            else
            {
                if (validationRayVisualization && visualizeValidSegments)
                {
                    validationRayVisualization(probes[prev].influence.center,
                                               probes[current].influence.center, false, userData);
                }
            }

            if (prev == start)
                break;

            auto nextPath = bakedPathData.lookupShortestPath(start, prev, nullptr);
            if (!nextPath.isValid())
            {
                return true;
            }

            current = prev;
            prev = (nextPath.direct) ? start : nextPath.lastProbe;
        }

        return false;
    }
    else
    {
        return false;
    }
}

// First, find the source-probe (the probe nearest to the source), and the listener-probes (all probes which influence
// the listener). For each listener-probe, query the baked data for the shortest path from the source-probe to the
// listener-probe.
//
// Weights are calculated for the paths reaching each listener-probe, such that if the listener is closer to a given
// listener-probe, its corresponding weight is larger.
//
// Optionally, we validate paths, by testing rays between every consecutive pair of probes.
//
// Optionally, if a baked path is found to be invalid (typically due to the presence of dynamic occluders), we search
// for alternate paths.
//
// Optionally, if the source and listener are in line of sight (this visibility check is a single ray cast), we create
// a SoundPath describing this.
//
// Finally, all the paths that haven't been discarded are weighted and summed into a set of SH and EQ coefficients.
bool PathSimulator::findPaths(const Vector3f& source,
                              const Vector3f& listener,
                              const IScene& scene,
                              const ProbeBatch& probes,
                              const ProbeNeighborhood& sourceProbes,
                              const ProbeNeighborhood& listenerProbes,
                              float radius,
                              float threshold,
                              float visRange,
                              int order,
                              bool enableValidation,
                              bool findAlternatePaths,
                              bool simplifyPaths,
                              bool realTimeVis,
                              float* eqGains,
                              float* coeffs,
                              const DistanceAttenuationModel& distanceAttenuationModel,
                              const DeviationModel& deviationModel,
                              Vector3f* avgDirection,
                              float* distanceRatio,
                              float* totalDeviation,
                              ValidationRayVisualizationCallback validationRayVisualization,
                              void* userData,
                              bool forceDirectOcclusion)
{
    PROFILE_FUNCTION();

    int numPaths = 0;
    SoundPath paths[kMaxPaths];
    float pathWeights[kMaxPaths];
    int starts[kMaxPaths];
    int ends[kMaxPaths];

    if (scene.isOccluded(listener, source) || forceDirectOcclusion)
    {
        if (sourceProbes.hasValidProbes() && listenerProbes.hasValidProbes())
        {
            BakedDataIdentifier identifier;
            identifier.variation = BakedDataVariation::Dynamic;
            identifier.type = BakedDataType::Pathing;

            if (probes.hasData(identifier))
            {
                const auto& bakedPathData = static_cast<const BakedPathData&>(probes[identifier]);

                if (sEnablePathsFromAllSourceProbes)
                {
                    for (auto i = 0; i < sourceProbes.numProbes(); ++i)
                    {
                        findPathsFromSourceProbe(source, listener, scene, probes, sourceProbes, listenerProbes, bakedPathData, i, sourceProbes.weights[i],
                                                 radius, threshold, visRange, enableValidation, findAlternatePaths, simplifyPaths, realTimeVis,
                                                 validationRayVisualization, userData, numPaths, paths, pathWeights, starts, ends);
                    }
                }
                else
                {
                    findPathsFromSourceProbe(source, listener, scene, probes, sourceProbes, listenerProbes, bakedPathData, sourceProbes.findNearest(source), 1.0f,
                                             radius, threshold, visRange, enableValidation, findAlternatePaths, simplifyPaths, realTimeVis,
                                             validationRayVisualization, userData, numPaths, paths, pathWeights, starts, ends);
                }
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        SoundPath directPath;
        directPath.direct = true;

        mResolvedPaths[numPaths].bends.clear();
        mResolvedPaths[numPaths].length = (source - listener).length();
        mResolvedPaths[numPaths].direction = Vector3f::unitVector(source - listener);
        paths[numPaths] = directPath;
        pathWeights[numPaths] = 1.0f;
        starts[numPaths] = -1;
        ends[numPaths] = -1;
        ++numPaths;
    }

    calcAmbisonicsCoeffsForPaths(numPaths, paths, pathWeights, order, distanceAttenuationModel, coeffs);

    calcEQForPaths(starts, ends, numPaths, paths, pathWeights, deviationModel, mResolvedPaths.data(), eqGains, totalDeviation);

    calcAverageDirectionForPaths(numPaths, paths, pathWeights, avgDirection);

    calcDistanceRatioForPaths(source, listener, numPaths, paths, pathWeights, distanceRatio);

    return true;
}

void PathSimulator::findPathsFromSourceProbe(const Vector3f& source,
                                             const Vector3f& listener,
                                             const IScene& scene,
                                             const ProbeBatch& probes,
                                             const ProbeNeighborhood& sourceProbes,
                                             const ProbeNeighborhood& listenerProbes,
                                             const BakedPathData& bakedPathData,
                                             int sourceProbeNeighborhoodIndex,
                                             float sourceProbeWeight,
                                             float radius,
                                             float threshold,
                                             float visRange,
                                             bool enableValidation,
                                             bool findAlternatePaths,
                                             bool simplifyPaths,
                                             bool realTimeVis,
                                             ValidationRayVisualizationCallback validationRayVisualization,
                                             void* userData,
                                             int& numPaths,
                                             SoundPath* paths,
                                             float* pathWeights,
                                             int* starts,
                                             int* ends)
{
    if (!sourceProbes.batches[sourceProbeNeighborhoodIndex] || sourceProbes.probeIndices[sourceProbeNeighborhoodIndex] < 0 || sourceProbes.batches[sourceProbeNeighborhoodIndex] != &probes)
        return;

    auto sourceProbeIndex = sourceProbes.probeIndices[sourceProbeNeighborhoodIndex];

    for (auto i = 0; i < listenerProbes.numProbes(); ++i)
    {
        findPathsFromSourceProbeToListenerProbe(source, listener, scene, probes, listenerProbes, bakedPathData, sourceProbeIndex, sourceProbeWeight, i,
                                                radius, threshold, visRange, enableValidation, findAlternatePaths,
                                                simplifyPaths, realTimeVis, validationRayVisualization, userData,
                                                numPaths, paths, pathWeights, starts, ends);
    }
}

void PathSimulator::findPathsFromSourceProbeToListenerProbe(const Vector3f& source,
                                                            const Vector3f& listener,
                                                            const IScene& scene,
                                                            const ProbeBatch& probes,
                                                            const ProbeNeighborhood& listenerProbes,
                                                            const BakedPathData& bakedPathData,
                                                            int sourceProbeIndex,
                                                            float sourceProbeWeight,
                                                            int listenerProbeNeighborhoodIndex,
                                                            float radius,
                                                            float threshold,
                                                            float visRange,
                                                            bool enableValidation,
                                                            bool findAlternatePaths,
                                                            bool simplifyPaths,
                                                            bool realTimeVis,
                                                            ValidationRayVisualizationCallback validationRayVisualization,
                                                            void* userData,
                                                            int& numPaths,
                                                            SoundPath* paths,
                                                            float* pathWeights,
                                                            int* starts,
                                                            int* ends)
{
    if (!listenerProbes.batches[listenerProbeNeighborhoodIndex] || listenerProbes.probeIndices[listenerProbeNeighborhoodIndex] < 0 || listenerProbes.batches[listenerProbeNeighborhoodIndex] != &probes)
        return;

    auto listenerProbeIndex = listenerProbes.probeIndices[listenerProbeNeighborhoodIndex];

    SoundPath soundPath;
    auto tryRealTime = false;

    mProbePath.reset();
    soundPath = bakedPathData.lookupShortestPath(sourceProbeIndex, listenerProbeIndex, &mProbePath);
    if (soundPath.isValid())
    {
        if (isPathOccluded(soundPath, scene, probes, radius, threshold, sourceProbeIndex,
            listenerProbeIndex, enableValidation, validationRayVisualization, userData, false))
        {
            soundPath = SoundPath(); // Discard the occluded path before trying an alternative.
            if (findAlternatePaths)
            {
                tryRealTime = true;
            }
        }
    }

    if (tryRealTime)
    {
        mProbePath = mPathFinder.findShortestPath(scene, probes, bakedPathData.visGraph(),
                                                 mVisTester, sourceProbeIndex, listenerProbeIndex, radius,
                                                 threshold, visRange, simplifyPaths, realTimeVis);

        soundPath = SoundPath(mProbePath, probes);
    }

    if (soundPath.isValid() && mProbePath.valid && numPaths < kMaxPaths)
    {
        // Include source and listener connections before simplifying the probe route.
        mRoutePoints.clear();
        auto append = [&](const Vector3f& point) {
            if (mRoutePoints.empty() || (point - mRoutePoints.back()).length() > 1e-5f)
                mRoutePoints.push_back(point);
        };
        append(source);
        append(probes[sourceProbeIndex].influence.center);
        for (auto node : mProbePath.nodes)
            append(probes[node].influence.center);
        append(probes[listenerProbeIndex].influence.center);
        append(listener);

        if (!mResolvedPaths[numPaths].resolve(mRoutePoints,
                [&](const Vector3f& a, const Vector3f& b) { return scene.isOccluded(a, b); }))
            return;
        if (validationRayVisualization)
        {
            for (auto i = size_t{1}; i < mRoutePoints.size(); ++i)
                validationRayVisualization(mRoutePoints[i - 1], mRoutePoints[i], false, userData);
        }
        paths[numPaths] = soundPath;
        pathWeights[numPaths] = sourceProbeWeight * listenerProbes.weights[listenerProbeNeighborhoodIndex];
        starts[numPaths] = sourceProbeIndex;
        ends[numPaths] = listenerProbeIndex;
        ++numPaths;
    }
}

SoundPath PathSimulator::findShortestPathFromSourceProbeToListenerProbe(const IScene& scene, const ProbeBatch& probes,
    int sourceProbeIndex, int listenerProbeIndex, const BakedPathData& bakedPathData, float radius, float threshold,
    float visRange, bool enableValidation, bool findAlternatePaths, bool simplifyPaths, bool realTimeVis,
    ProbePath& probePath, ValidationRayVisualizationCallback validationRayVisualization, void* userData)
{
    if (sourceProbeIndex == listenerProbeIndex)
    {
        SoundPath soundPath{};
        soundPath.direct = true;

        return soundPath;
    }

    auto tryRealTime = false;
    auto soundPath = bakedPathData.lookupShortestPath(sourceProbeIndex, listenerProbeIndex, &probePath);
    if (soundPath.isValid())
    {
        if (isPathOccluded(soundPath, scene, probes, radius, threshold, sourceProbeIndex,
            listenerProbeIndex, enableValidation, validationRayVisualization, userData))
        {
            if (findAlternatePaths)
            {
                tryRealTime = true;
            }
        }
    }

    if (tryRealTime)
    {
        probePath = mPathFinder.findShortestPath(scene, probes, bakedPathData.visGraph(),
            mVisTester, sourceProbeIndex, listenerProbeIndex, radius,
            threshold, visRange, simplifyPaths, realTimeVis);

        soundPath = SoundPath(probePath, probes);
    }

    return soundPath;
}

// Project resolved arrival directions into Ambisonics, weighted by path weight and distance attenuation.
void PathSimulator::calcAmbisonicsCoeffsForPaths(int numPaths,
                                                 const SoundPath* paths,
                                                 const float* weights,
                                                 int order,
                                                 const DistanceAttenuationModel& distanceAttenuationModel,
                                                 float* coeffs)
{
    PROFILE_FUNCTION();

    if (coeffs)
    {
        auto numCoeffs = SphericalHarmonics::numCoeffsForOrder(order);
        memset(coeffs, 0, numCoeffs * sizeof(float));

        for (auto i = 0; i < numPaths; ++i)
        {
            if (!paths[i].isValid())
                continue;

            const auto distanceAttenuation = distanceAttenuationModel.evaluate(mResolvedPaths[i].length);
            const auto gain = weights[i] * distanceAttenuation;
            const auto& direction = mResolvedPaths[i].direction;
            SphericalHarmonics::projectSinglePointAndUpdate(direction, order, gain, coeffs);
        }
    }
}

void PathSimulator::calcEQForPaths(int* starts,
                                   int* ends,
                                   int numPaths,
                                   const SoundPath* paths,
                                   const float* weights,
                                   const DeviationModel& deviationModel,
                                   const ResolvedPath* resolvedPaths,
                                   float* eqGains,
                                   float* totalDeviation)
{
    PROFILE_FUNCTION();

    if (eqGains)
    {
        memset(eqGains, 0, Bands::kNumBands * sizeof(float));
        auto numValidPaths = 0;

        for (auto i = 0; i < numPaths; ++i)
        {
            if (!paths[i].isValid())
                continue;

            if (starts[i] >= 0 && ends[i] >= 0)
            {
                float deviationTerm[Bands::kNumBands];
                for (auto j = 0; j < Bands::kNumBands; ++j)
                {
                    deviationTerm[j] = deviationModel.evaluatePath(resolvedPaths[i].bends.data(),
                        static_cast<int>(resolvedPaths[i].bends.size()), j);
                }

                auto overallGain = 1.0f;
                EQEffect::normalizeGains(deviationTerm, overallGain);

                for (auto j = 0; j < Bands::kNumBands; ++j)
                {
                    eqGains[j] += (weights[i] * overallGain * deviationTerm[j]);
                }
            }
            else
            {
                for (auto j = 0; j < Bands::kNumBands; ++j)
                {
                    eqGains[j] += weights[i];
                }
            }

            ++numValidPaths;
        }

        if (numValidPaths == 0)
        {
            for (auto i = 0; i < Bands::kNumBands; ++i)
            {
                eqGains[i] = 1.0f;
            }
        }
    }

    if (totalDeviation)
    {
        *totalDeviation = .0f;

        for (auto i = 0; i < numPaths; ++i)
        {
            if (!paths[i].isValid())
                continue;

            if (starts[i] >= 0 && ends[i] >= 0)
            {
                for (auto bend : resolvedPaths[i].bends)
                    *totalDeviation += weights[i] * bend;
            }
        }
    }
}


void PathSimulator::calcAverageDirectionForPaths(int numPaths,
                                                 const SoundPath* paths,
                                                 const float* weights,
                                                 Vector3f* avgDirection)
{
    PROFILE_FUNCTION();

    if (!avgDirection)
        return;

    Vector3f direction(.0f, .0f, .0f);
    for (auto i = 0; i < numPaths; ++i)
    {
        if (!paths[i].isValid())
            continue;

        const auto distanceAttenuation = 1.0f / std::max(mResolvedPaths[i].length, 1.0f);
        const auto gain = weights[i] * distanceAttenuation;
        const auto& pathDirection = mResolvedPaths[i].direction;

        direction += (pathDirection * gain);
    }

    *avgDirection = Vector3f::unitVector(direction);
}

void PathSimulator::calcDistanceRatioForPaths(const Vector3f& source,
                                              const Vector3f& listener,
                                              int numPaths,
                                              const SoundPath* paths,
                                              const float* weights,
                                              float* avgDistanceRatio)
{
    PROFILE_FUNCTION();

    if (!avgDistanceRatio)
        return;

    float ratio = .0f;
    for (auto i = 0; i < numPaths; ++i)
    {
        if (!paths[i].isValid())
            continue;

        const auto pathDistance = mResolvedPaths[i].length;
        const auto directDistance = (source - listener).length();
        const auto pathRatio = (pathDistance > 1.0f && directDistance > 1.0f)
            ? directDistance / pathDistance : 1.0f;

        ratio += weights[i] * pathRatio;
    }

    // Returns ratio of 1.0 if no valid path is found. Probably should return a large distance ratio instead.
    *avgDistanceRatio = ratio;
}

}
