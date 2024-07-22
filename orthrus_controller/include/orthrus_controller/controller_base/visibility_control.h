#ifndef ORTHRUS_CONTROLLER_VISIBILITY_CONTROL_H
#define ORTHRUS_CONTROLLER_VISIBILITY_CONTROL_H


// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ORTHRUS_CONTROLLER_EXPORT __attribute__((dllexport))
#define ORTHRUS_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define ORTHRUS_CONTROLLER_EXPORT __declspec(dllexport)
#define ORTHRUS_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef ORTHRUS_CONTROLLER_BUILDING_DLL
#define ORTHRUS_CONTROLLER_PUBLIC ORTHRUS_CONTROLLER_EXPORT
#else
#define ORTHRUS_CONTROLLER_PUBLIC ORTHRUS_CONTROLLER_IMPORT
#endif
#define ORTHRUS_CONTROLLER_PUBLIC_TYPE ORTHRUS_CONTROLLER_PUBLIC
#define ORTHRUS_CONTROLLER_LOCAL
#else
#define ORTHRUS_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define ORTHRUS_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define ORTHRUS_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define ORTHRUS_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define ORTHRUS_CONTROLLER_PUBLIC
#define ORTHRUS_CONTROLLER_LOCAL
#endif
#define ORTHRUS_CONTROLLER_PUBLIC_TYPE
#endif

#endif /* ORTHRUS_CONTROLLER_VISIBILITY_CONTROL_H */

