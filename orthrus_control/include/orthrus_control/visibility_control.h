#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ORTHRUS_TEST_CONTROL_EXPORT __attribute__((dllexport))
#define ORTHRUS_TEST_CONTROL_IMPORT __attribute__((dllimport))
#else
#define ORTHRUS_TEST_CONTROL_EXPORT __declspec(dllexport)
#define ORTHRUS_TEST_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef ORTHRUS_TEST_CONTROL_BUILDING_DLL
#define ORTHRUS_TEST_CONTROL_PUBLIC ORTHRUS_TEST_CONTROL_EXPORT
#else
#define ORTHRUS_TEST_CONTROL_PUBLIC ORTHRUS_TEST_CONTROL_IMPORT
#endif
#define ORTHRUS_TEST_CONTROL_PUBLIC_TYPE ORTHRUS_TEST_CONTROL_PUBLIC
#define ORTHRUS_TEST_CONTROL_LOCAL
#else
#define ORTHRUS_CONTROL_EXPORT __attribute__((visibility("default")))
#define ORTHRUS_TEST_CONTROL_IMPORT
#if __GNUC__ >= 4
#define ORTHRUS_TEST_CONTROL_PUBLIC __attribute__((visibility("default")))
#define ORTHRUS_TEST_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define ORTHRUS_TEST_CONTROL_PUBLIC
#define ORTHRUS_TEST_CONTROL_LOCAL
#endif
#define ORTHRUS_TEST_CONTROL_PUBLIC_TYPE
#endif