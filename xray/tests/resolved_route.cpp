#ifdef NDEBUG
#error Test assertions must remain enabled
#endif
#include "resolved_path.h"
#include "deviation.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
namespace ipl
{
Memory &gMemory()
{
    static Memory memory{};
    return memory;
}
} // namespace ipl
using namespace ipl;
int main()
{
    ResolvedPath r;
    auto clear = [](const Vector3f &, const Vector3f &) { return false; };
    vector<Vector3f> straight{{0, 0, 0}, {1, 1, 0}, {2, -1, 0}, {3, 0, 0}};
    assert(r.resolve(straight, clear));
    assert(straight.size() == 2 && r.bends.empty());
    assert(std::abs(r.length - 3) < 1e-6 && r.direction.x() < -.999f);
    // A wall on x=0 ending at y=1: source and listener on opposite sides.
    auto wall = [](const Vector3f &a, const Vector3f &b) {
        if (a.x() * b.x() >= 0)
            return false;
        float t = -a.x() / (b.x() - a.x());
        return a.y() + t * (b.y() - a.y()) < 1.f;
    };
    vector<Vector3f> around{{-2, 0, 0}, {-1, 1.5, 0}, {1, 1.5, 0}, {2, 0, 0}};
    assert(r.resolve(around, wall));
    assert(around.size() >= 3 && r.bends.size() == around.size() - 2);
    assert(r.length > 4 && r.direction.x() < 0 && r.direction.y() > 0);
    for (size_t i = 1; i < around.size(); ++i)
        assert(!wall(around[i - 1], around[i]));
    // The first segment crosses the wall, although the remaining probes can see each other.
    vector<Vector3f> bad{{-2, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    assert(!r.resolve(bad, wall));
    assert(r.bends.empty() && r.length == 0);
    // A detour made entirely of clear probe samples must not accumulate edge loss.
    DeviationModel model;
    vector<Vector3f> open{{0, 0, 0}, {1, 2, 0}, {2, -2, 0}, {4, 0, 0}};
    assert(r.resolve(open, clear));
    for (int band = 0; band < 3; ++band)
        assert(std::abs(model.evaluatePath(r.bends.data(), r.bends.size(), band) - 1) < 1e-6);
    // Near the wall edge, remove excess loss caused by the probe detour.
    vector<Vector3f> nearEdge{{-2, 0, 0}, {-1, 2, 0}, {1, 2, 0}, {2, 1.95f, 0}};
    assert(r.resolve(nearEdge, wall));
    for (size_t i = 1; i < nearEdge.size(); ++i)
        assert(!wall(nearEdge[i - 1], nearEdge[i]));
    assert(model.evaluatePath(r.bends.data(), r.bends.size(), 1) > .85f);
    vector<Vector3f> behindListener{{-2, 0, 0}, {2.05f, 2.1f, 0}, {2, 1.95f, 0}};
    assert(r.resolve(behindListener, wall));
    assert(model.evaluatePath(r.bends.data(), r.bends.size(), 1) > .85f);
    assert(r.direction.x() < 0); // The last segment must approach from the wall side.
    vector<Vector3f> invalid{{0, 0, 0}, {std::numeric_limits<float>::quiet_NaN(), 0, 0}};
    assert(!r.resolve(invalid, clear));
    puts("SDK_RESOLVED_PASS");
}
