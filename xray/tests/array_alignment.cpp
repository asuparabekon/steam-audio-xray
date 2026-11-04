#include "array_math.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#ifdef NDEBUG
#error Assertions must be enabled
#endif
int main()
{
    // All combinations of aligned/8-byte-offset complex buffers, with SIMD tails.
    for (int a = 0; a < 2; ++a)
        for (int b = 0; b < 2; ++b)
            for (int c = 0; c < 2; ++c)
                for (int n = 0; n <= 17; ++n)
                {
                    alignas(16) ipl::complex_t x[20], y[20], z[20], expected[20];
                    for (int i = 0; i < 20; ++i)
                    {
                        x[i] = {float(i) * .13f - .9f, float(i) * -.07f + .4f};
                        y[i] = {float(i) * -.11f + .7f, float(i) * .09f - .3f};
                        z[i] = expected[i] = {.31f, -.27f};
                    }
                    for (int i = 0; i < n; ++i)
                        expected[c + i] += x[a + i] * y[b + i];
                    ipl::ArrayMath::multiplyAccumulate(n, x + a, y + b, z + c);
                    for (int i = 0; i < 20; ++i)
                        assert(std::abs(z[i] - expected[i]) < 1e-5f);
                }
    puts("SDK_ARRAY_ALIGNMENT_PASS");
}
