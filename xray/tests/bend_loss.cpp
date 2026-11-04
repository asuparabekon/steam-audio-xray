#ifdef NDEBUG
#error Test assertions must remain enabled
#endif
#include "deviation.h"
#include "math_functions.h"
#include <cassert>
#include <cstdio>
#include <cmath>
float custom(float angle, int, void *)
{
    return 2.f / (1.f + angle);
}
float zeroOrigin(float angle, int, void *)
{
    return angle < .001f ? 0.f : .5f;
}
int main()
{
    ipl::DeviationModel model;
    const float right = ipl::Math::kPi * .5f;
    float bends[] = {right, right, right, right};
    float straight[] = {0, right, 0, right, 0};
    for (int band = 0; band < 3; ++band)
    {
        const float single = model.evaluate(right, band) / model.evaluate(1e-8f, band);
        assert(std::abs(model.evaluatePath(bends, 1, band) - single) < 1e-6f);
        assert(model.evaluatePath(nullptr, 0, band) == 1.f);
        assert(std::abs(model.evaluatePath(straight, 5, band) - single * single) < 1e-6f);
        float previous = 1;
        for (int count = 1; count <= 4; ++count)
        {
            float value = model.evaluatePath(bends, count, band);
            assert(value < previous);
            previous = value;
        }
        printf("band=%d one_corner=%g four_corners=%g old_near_full_turn=%g\n", band, single,
               model.evaluatePath(bends, 4, band),
               model.evaluate(4 * right + .001f, band) / model.evaluate(1e-8f, band));
    }
    ipl::DeviationModel callback(custom, nullptr);
    assert(std::abs(callback.evaluatePath(bends, 4, 0) - custom(4 * right, 0, nullptr)) < 1e-6f);
    assert(callback.evaluatePath(nullptr, 0, 0) == custom(0, 0, nullptr));
    ipl::DeviationModel zeroCallback(zeroOrigin, nullptr);
    assert(zeroCallback.evaluatePath(bends, 1, 0) == .5f);
    assert(zeroCallback.evaluatePath(nullptr, 0, 0) == 0.f);
    puts("SDK_BEND_PASS");
}
