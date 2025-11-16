#include "Ray_Tracer.h"
#include <stdio.h>

Ray_Tracer::Ray_Tracer(void)
{
    // Scene dimensions
    float dim = 512.0f;
    _dim[0] = dim; _dim[1] = dim; _dim[2] = dim;

    // Build scene
    _scene.set_dim(_dim);
    _scene.assemble();

    // Setup view plane (orthonormal basis, pinhole eye)
    _view_plane.set_origin(0.0f, 0.0f, _dim[2]);
    _view_plane.set_u(1.0f, 0.0f, 0.0f);
    _view_plane.set_v(0.0f, 1.0f, 0.0f);
    _view_plane.set_eye(_dim[0] / 2.0f, _dim[1] / 2.0f, _dim[2] + 2000.0f);
}

Ray_Tracer::~Ray_Tracer(void)
{
}

void Ray_Tracer::run(Image& image)
{
    // Image buffer setup
    image.ncolorChannels = 3;
    image.nx = (int)_dim[0];
    image.ny = (int)_dim[1];
    image.n = image.nx * image.ny * image.ncolorChannels;

    image.data = new unsigned char[image.n];
    image.fdata = new float[image.n];

    // Ray gen / color buffers
    M3DVector3f ray;
    M3DVector3f color;
    M3DVector3f pij;

    printf("Start Ray Tracing (local shading only)...\n");
    int lastPercent = -1;

    for (int j = 0; j < image.ny; ++j)
    {
        for (int i = 0; i < image.nx; ++i)
        {
            // Pixel sample on view plane, then primary ray
            _view_plane.get_pij(pij, (float)i, (float)j);
            _view_plane.get_per_ray(ray, pij);

            // Local Phong shading only (no recursion)
            ray_tracing(pij, ray, color);

            const unsigned int idx = (j * image.nx + i) * 3u;
            image.fdata[idx + 0] = color[0];
            image.fdata[idx + 1] = color[1];
            image.fdata[idx + 2] = color[2];
        }

        int percent = (int)((j + 1) * 100.0f / image.ny);
        if (percent != lastPercent) {
            lastPercent = percent;
            printf("\rProgress: %3d%%", percent);
            fflush(stdout);
        }
    }
    printf("\nRay Tracing Finished!\n");

    // Normalize to 0..255
    float max_v = 0.0f;
    for (int k = 0; k < image.n; ++k)
        if (image.fdata[k] > max_v) max_v = image.fdata[k];

    if (max_v < 1e-8f) max_v = 1.0f; // avoid divide-by-zero; produce black

    for (int k = 0; k < image.n; ++k)
    {
        float v = (image.fdata[k] / max_v) * 255.0f;
        if (v < 0.0f) v = 0.0f;
        if (v > 255.0f) v = 255.0f;
        image.data[k] = (unsigned char)(v);
    }
}

void Ray_Tracer::ray_tracing(M3DVector3f start,
    M3DVector3f direct,
    M3DVector3f color)
{
    // Normalize ray direction
    m3dNormalizeVector(direct);

    // Find closest hit
    Basic_Primitive* prim = NULL;
    M3DVector3f hitPoint;
    if (_scene.intersection_check(start, direct, &prim, hitPoint) != _k_miss)
    {
        // Ambient light
        M3DVector3f am_light;
        _scene.get_amb_light(am_light);

        // Hard shadow test (point to light)
        bool shadow = check_shadow(hitPoint);

        // Local Phong shading
        prim->shade(direct, hitPoint, _scene.get_sp_light(), am_light, color, shadow);
    }
    else
    {
        // Background color: black
        m3dLoadVector3(color, 0.0f, 0.0f, 0.0f);
    }
}

bool Ray_Tracer::check_shadow(M3DVector3f intersect_point)
{
    const Light& light = _scene.get_sp_light();

    // Vector from hit point to light
    M3DVector3f Lpos; light.get_light_pos(Lpos);
    M3DVector3f toLight;
    m3dSubtractVectors3(toLight, Lpos, intersect_point);
    float dist2 = m3dDotProduct(toLight, toLight);
    m3dNormalizeVector(toLight);

    // Offset origin slightly along the shadow ray to avoid acne
    M3DVector3f origin;
    m3dCopyVector3(origin, intersect_point);
    M3DVector3f eps; m3dCopyVector3(eps, toLight); m3dScaleVector3(eps, 1e-3f);
    m3dAddVectors3(origin, origin, eps);

    // Trace toward the light
    Basic_Primitive* blocker = NULL;
    M3DVector3f pHit;
    if (_scene.intersection_check(origin, toLight, &blocker, pHit) == _k_hit)
    {
        // If blocker is between point and light, it's in shadow
        M3DVector3f remain;
        m3dSubtractVectors3(remain, Lpos, pHit);
        float newDist2 = m3dDotProduct(remain, remain);
        return (newDist2 < dist2);
    }
    return false;
}
