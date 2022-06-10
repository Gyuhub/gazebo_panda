#ifndef HQP_CONTROLLER__VISIBILITY_CONTROL_H_
#define HQP_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HQP_CONTROLLER_EXPORT __attribute__((dllexport))
#define HQP_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define HQP_CONTROLLER_EXPORT __declspec(dllexport)
#define HQP_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef HQP_CONTROLLER_BUILDING_DLL
#define HQP_CONTROLLER_PUBLIC HQP_CONTROLLER_EXPORT
#else
#define HQP_CONTROLLER_PUBLIC HQP_CONTROLLER_IMPORT
#endif
#define HQP_CONTROLLER_PUBLIC_TYPE HQP_CONTROLLER_PUBLIC
#define HQP_CONTROLLER_LOCAL
#else
#define HQP_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define HQP_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define HQP_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define HQP_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define HQP_CONTROLLER_PUBLIC
#define HQP_CONTROLLER_LOCAL
#endif
#define HQP_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // HQP_CONTROLLER__VISIBILITY_CONTROL_H_