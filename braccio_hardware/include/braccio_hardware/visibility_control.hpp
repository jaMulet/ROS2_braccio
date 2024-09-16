 #ifndef VISIBILITY_CONTROL_HPP_
 #define VISIBILITY_CONTROL_HPP_
 
 // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
 //     https://gcc.gnu.org/wiki/Visibility
 
 #if defined _WIN32 || defined __CYGWIN__
   #ifdef __GNUC__
     #define BRACCIO_HARDWARE_EXPORT __attribute__ ((dllexport))
     #define BRACCIO_HARDWARE_IMPORT __attribute__ ((dllimport))
   #else
     #define BRACCIO_HARDWARE_EXPORT __declspec(dllexport)
     #define BRACCIO_HARDWARE_IMPORT __declspec(dllimport)
   #endif
   #ifdef BRACCIO_HARDWARE_BUILDING_DLL
     #define BRACCIO_HARDWARE_PUBLIC BRACCIO_HARDWARE_EXPORT
   #else
     #define BRACCIO_HARDWARE_PUBLIC BRACCIO_HARDWARE_IMPORT
   #endif
   #define BRACCIO_HARDWARE_PUBLIC_TYPE BRACCO_HARDWARE_PUBLIC
   #define BRACCIO_HARDWARE_LOCAL
 #else
   #define BRACCIO_HARDWARE_EXPORT __attribute__ ((visibility("default")))
   #define BRACCIO_HARDWARE_IMPORT
   #if __GNUC__ >= 4
     #define BRACCIO_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
     #define BRACCIO_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
   #else
     #define BRACCIO_HARDWARE_PUBLIC
     #define BRACCIO_HARDWARE_LOCAL
   #endif
   #define BRACCIO_HARDWARE_PUBLIC_TYPE
 #endif
 
 // based on wiki.ros.org: http://wiki.ros.org/win_ros/Contributing/Dll%20Exports
 // Ignore warnings about import/exports when deriving from std classes.
 // https://docs.microsoft.com/en-us/cpp/error-messages/compiler-warnings/compiler-warning-level-1-c4251
 // https://docs.microsoft.com/en-us/cpp/error-messages/compiler-warnings/compiler-warning-level-2-c4275
 #ifdef _MSC_VER
   #pragma warning(disable: 4251)
   #pragma warning(disable: 4275)
 #endif
 
 #endif  // VISIBILITY_CONTROL_HPP_