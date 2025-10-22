#ifndef D1_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define D1_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define D1_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define D1_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define D1_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define D1_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef D1_HARDWARE_INTERFACE_BUILDING_DLL
#define D1_HARDWARE_INTERFACE_PUBLIC D1_HARDWARE_INTERFACE_EXPORT
#else
#define D1_HARDWARE_INTERFACE_PUBLIC D1_HARDWARE_INTERFACE_IMPORT
#endif
#define D1_HARDWARE_INTERFACE_PUBLIC_TYPE D1_HARDWARE_INTERFACE_PUBLIC
#define D1_HARDWARE_INTERFACE_LOCAL
#else
#define D1_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define D1_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define D1_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define D1_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define D1_HARDWARE_INTERFACE_PUBLIC
#define D1_HARDWARE_INTERFACE_LOCAL
#endif
#define D1_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // D1_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_