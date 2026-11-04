cmake_minimum_required(VERSION 3.17)
project(SteamAudioRouteTests LANGUAGES CXX)
set(STEAMAUDIO_SOURCE "" CACHE PATH "Patched Steam Audio checkout root")
set(core "${STEAMAUDIO_SOURCE}/core/src/core")
if(NOT EXISTS "${core}/resolved_path.h")
    message(FATAL_ERROR "STEAMAUDIO_SOURCE must name the patched SDK checkout")
endif()
enable_testing()
add_executable(array_alignment
    array_alignment.cpp
    "${core}/array_math.cpp")
add_executable(bend_loss
    bend_loss.cpp
    "${core}/deviation.cpp"
    "${core}/bands.cpp"
    "${core}/math_functions.cpp")
add_executable(resolved_route
    resolved_route.cpp
    "${core}/deviation.cpp"
    "${core}/bands.cpp"
    "${core}/math_functions.cpp"
    "${core}/memory_allocator.cpp")
foreach(test array_alignment bend_loss resolved_route)
    target_compile_features(${test} PRIVATE cxx_std_17)
    target_include_directories(${test} PRIVATE "${core}")
    if(MSVC)
        target_compile_options(${test} PRIVATE /UNDEBUG)
        target_compile_definitions(${test} PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${test} PRIVATE -UNDEBUG)
    endif()
    add_test(NAME ${test} COMMAND ${test})
endforeach()
# AVX permits unaligned arithmetic operands and can hide the original SSE fault.
if(NOT MSVC AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|i.86)$")
    target_compile_options(array_alignment PRIVATE -msse2 -mno-avx)
endif()
