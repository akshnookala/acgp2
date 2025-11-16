#pragma once
#include "scene/Scene.h"
#include "scene/view_plane.h"
#include "common/image_volume.h"

class Ray_Tracer
{
public:
    Ray_Tracer(void);
    ~Ray_Tracer(void);

    // Render the image (local Phong shading only)
    void run(Image& image);

private:
    // Local shading only: start, direction, output color
    void ray_tracing(M3DVector3f start, M3DVector3f direct, M3DVector3f color);

    // Shadow test from hit point toward the point light
    bool check_shadow(M3DVector3f intersect_point);

private:
    Scene       _scene;
    View_Plane  _view_plane;
    M3DVector3f _dim;
};
