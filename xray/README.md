# Steam Audio for X-Ray

Based on Steam Audio 4.8.1. This fork changes how baked probe paths are used
for sound direction and attenuation in X-Ray.

## Pathing changes

Probe centers can add turns that do not correspond to an obstacle. This fork
removes those detours with visibility checks, then moves the remaining points
toward shorter routes while keeping both adjoining segments clear. It includes
the source and listener connections and uses the resulting route for distance,
arrival direction and attenuation.

The default diffraction model applies loss at each bend. Previously, it used
the sum of the bend angles, which could give winding paths too little loss.
Custom deviation callbacks still receive the total angle. The debug callback
shows the resolved route, including its source and listener connections.

Path refinement uses the existing probe graph; it cannot find a route through
an opening the probes do not cover. The baked data format is unchanged.

## SSE fix

The non-IPP complex multiply-accumulate routine now uses an unaligned load for
the output buffer, matching its unaligned store. The aligned load could crash
when the buffer started at an 8-byte offset from a 16-byte boundary.

## Building

Follow the SDK [build instructions](../core/doc/build-instructions.rst).
The Windows build used for X-Ray uses MSVC 14.44, Embree 4.4 and PFFFT
(`STEAMAUDIO_ENABLE_EMBREE=ON`, `STEAMAUDIO_ENABLE_IPP=OFF`).

## Tests

From the repository root, using a C++17 compiler:

```sh
cmake -S xray/tests -B build/xray-tests \
  -DSTEAMAUDIO_SOURCE="$PWD" -DCMAKE_BUILD_TYPE=Release
cmake --build build/xray-tests --config Release --parallel 1
ctest --test-dir build/xray-tests -C Release --output-on-failure
```

The tests cover buffer alignment, bend loss, custom callbacks, route visibility
and arrival direction. Assertions stay enabled in Release builds. The alignment
test disables AVX on GCC/Clang x86 builds because AVX can hide the SSE alignment
fault; use the default SSE2 target with MSVC.
