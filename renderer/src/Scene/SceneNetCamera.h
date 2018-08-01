#include "Math/Vector3.h"
#include <optixu/optixu_math_namespace.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>

class SceneNetCamera {
public:
#ifdef __CUDACC__
    SceneNetCamera() {}
#else
    SceneNetCamera() {}
    SceneNetCamera(Vector3 eye, Vector3 lookat, Vector3 up, float hFoV=60, float vFoV=45);
    void init();
    void transform(const std::pair<TooN::Vector<3> ,TooN::Vector<3> >& start, const std::pair<TooN::Vector<3> ,TooN::Vector<3> >& end, const float interp);
#endif
    optix::float3 eye, lookat, up;
    optix::float3 lookdir, camera_u, camera_v;
    float ulen_, vlen_;
};
