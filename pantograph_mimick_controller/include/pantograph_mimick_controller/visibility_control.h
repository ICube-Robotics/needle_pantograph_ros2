/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef PANTOGRAPH_MIMICK_CONTROLLER__VISIBILITY_CONTROL_H_
#define PANTOGRAPH_MIMICK_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PANTOGRAPH_MIMICK_CONTROLLER_EXPORT __attribute__((dllexport))
#define PANTOGRAPH_MIMICK_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define PANTOGRAPH_MIMICK_CONTROLLER_EXPORT __declspec(dllexport)
#define PANTOGRAPH_MIMICK_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef PANTOGRAPH_MIMICK_CONTROLLER_BUILDING_DLL
#define PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC PANTOGRAPH_MIMICK_CONTROLLER_EXPORT
#else
#define PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC PANTOGRAPH_MIMICK_CONTROLLER_IMPORT
#endif
#define PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC_TYPE PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
#define PANTOGRAPH_MIMICK_CONTROLLER_LOCAL
#else
#define PANTOGRAPH_MIMICK_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define PANTOGRAPH_MIMICK_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define PANTOGRAPH_MIMICK_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
#define PANTOGRAPH_MIMICK_CONTROLLER_LOCAL
#endif
#define PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // PANTOGRAPH_MIMICK_CONTROLLER__VISIBILITY_CONTROL_H_
