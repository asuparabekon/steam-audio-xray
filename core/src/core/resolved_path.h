// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "vector.h"
#include "containers.h"
#include "math_functions.h"

namespace ipl
{
// Route length, arrival direction and bends after visibility-based simplification.
struct ResolvedPath
{
    vector<float> bends;
    float length = 0.0f;
    Vector3f direction;

    // Updates points in place. The first and last points are the source and listener;
    // consecutive duplicates must be removed by the caller.
    template <typename Occluded> bool resolve(vector<Vector3f> &points, Occluded occluded)
    {
        length = 0.0f;
        direction = Vector3f(0.0f, 0.0f, 0.0f);
        bends.clear();
        if (points.size() < 2)
            return false;
        for (const auto &point : points)
        {
            if (!Math::isFinite(point.x()) || !Math::isFinite(point.y()) || !Math::isFinite(point.z()))
                return false;
        }
        // Connect each point to the farthest visible point remaining in the route.
        auto write = size_t{1};
        auto current = size_t{0};
        while (current + 1 < points.size())
        {
            auto next = points.size() - 1;
            while (next > current && occluded(points[current], points[next]))
                --next;
            if (next == current)
                return false;
            points[write++] = points[next];
            current = next;
        }
        points.resize(write);
        if (points.size() < 2)
            return false;

        // Move intermediate points toward the chord between their neighbors,
        // checking visibility of both segments after each proposed move.
        constexpr float spatialTolerance = 0.02f;
        for (auto pass = 0; pass < 4 && points.size() > 2; ++pass)
        {
            auto moved = false;
            for (auto step = size_t{1}; step + 1 < points.size(); ++step)
            {
                const auto i = (pass % 2 == 0) ? step : points.size() - 1 - step;
                const auto a = points[i - 1];
                const auto b = points[i + 1];
                const auto chord = b - a;
                const auto squaredLength = Vector3f::dot(chord, chord);
                if (squaredLength < 1e-8f)
                    continue;
                const auto t =
                    std::max(0.0f, std::min(1.0f, Vector3f::dot(points[i] - a, chord) / squaredLength));
                // Also try each neighbor: projection onto the chord alone can leave
                // a point behind the listener and distort the arrival direction.
                const Vector3f targets[] = {a + chord * t, a, b};
                for (const auto &target : targets)
                {
                    const auto delta = target - points[i];
                    const auto distance = delta.length();
                    if (distance <= spatialTolerance)
                        continue;
                    auto low = 0.0f;
                    auto high = 1.0f;
                    for (auto iteration = 0; iteration < 14 && (high - low) * distance > spatialTolerance;
                         ++iteration)
                    {
                        const auto fraction = (low + high) * 0.5f;
                        const auto candidate = points[i] + delta * fraction;
                        if (!occluded(a, candidate) && !occluded(candidate, b))
                            low = fraction;
                        else
                            high = fraction;
                    }
                    if (low * distance > spatialTolerance)
                    {
                        points[i] += delta * low;
                        moved = true;
                    }
                }
            }
            // A relaxed point can make an adjacent node redundant.
            for (auto i = size_t{1}; i + 1 < points.size();)
            {
                if (!occluded(points[i - 1], points[i + 1]))
                    points.erase(points.begin() + i);
                else
                    ++i;
            }
            if (!moved)
                break;
        }

        Vector3f previousDirection;
        for (auto i = size_t{1}; i < points.size(); ++i)
        {
            const auto segment = points[i] - points[i - 1];
            const auto segmentLength = segment.length();
            if (!Math::isFinite(segmentLength) || segmentLength <= 1e-5f)
                return false;
            const auto direction = segment / segmentLength;
            length += segmentLength;
            if (i > 1)
                bends.push_back(Vector3f::angleBetween(previousDirection, direction));
            previousDirection = direction;
        }
        this->direction = -previousDirection;
        return true;
    }
};
} // namespace ipl
