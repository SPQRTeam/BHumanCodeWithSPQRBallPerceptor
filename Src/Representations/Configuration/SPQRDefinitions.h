//   BALL PERCEPTION ----------------------------------------------------------------------------

#ifdef TARGET_ROBOT
  #define UPPER_CAMERA_WIDTH  320
  #define UPPER_CAMERA_HEIGHT 240
#else
  #define UPPER_CAMERA_WIDTH  640
  #define UPPER_CAMERA_HEIGHT 480
#endif
#ifdef TARGET_ROBOT
    #define LOWER_CAMERA_WIDTH  160
    #define LOWER_CAMERA_HEIGHT 120
#else
    #define LOWER_CAMERA_WIDTH  320
    #define LOWER_CAMERA_HEIGHT 240
#endif
#define UPPER_SCALE_FACTOR  (float)UPPER_CAMERA_WIDTH / 640.0f
#define LOWER_SCALE_FACTOR  (float)LOWER_CAMERA_WIDTH / 320.0f

// To use those macro you need to include the CameraInfo module
#define FROMBHUMANVI(v) (Vector2i(v.x()*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR),v.y()*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR)))
#define FROMBHUMANVS(v) (Vector2s(v.x()*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR),v.y()*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR)))
#define FROMBHUMANVF(v) (Vector2f(v.x()*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR),v.y()*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR)))
#define FROMBHUMANF(v) (v*((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR))
#define TOBHUMANVI(v) (Vector2i(v.x()/((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR),v.y()/((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR)))
#define TOBHUMANVS(v) (Vector2s(v.x()/((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR),v.y()/((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR)))
#define TOBHUMANF(v) (v/((theCameraInfo.camera==CameraInfo::upper)?UPPER_SCALE_FACTOR:LOWER_SCALE_FACTOR))
