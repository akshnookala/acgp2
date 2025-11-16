#include "Sphere.h"
#include "../common/math3d.h"
#include <math.h>
#include <algorithm>

// Ray–Sphere intersection (geometric)
Intersect_Cond Sphere::intersection_check(const M3DVector3f start,
    const M3DVector3f dir,
    float& distance,
    M3DVector3f intersection_p)
{
    // L = center - origin
    M3DVector3f L; m3dSubtractVectors3(L, _pos, start);
    float tca = m3dDotProduct(L, dir);
    if (tca < 0.0f) return _k_miss;

    float d2 = m3dDotProduct(L, L) - tca * tca;
    if (d2 > _rad2) return _k_miss;

    float thc = sqrtf(_rad2 - d2);
    distance = tca - thc;

    // hit point
    M3DVector3f step; m3dCopyVector3(step, dir); m3dScaleVector3(step, distance);
    m3dAddVectors3(intersection_p, start, step);
    return _k_hit;
}

// Phong local shading
void Sphere::shade(M3DVector3f view,
    M3DVector3f intersect_p,
    const Light& sp_light,
    M3DVector3f am_light,
    M3DVector3f color,
    bool shadow)
{
    const float ka = _ka;
    const float kd = _kd;
    const float ks = _ks;
    const float shininess = 20.0f;

    // Base color
    M3DVector3f base; m3dCopyVector3(base, _color);

    // Light data
    M3DVector3f light_pos, light_col; sp_light.get_light(light_pos, light_col);

    // Ambient
    for (int i = 0; i < 3; ++i) color[i] = ka * am_light[i] * base[i];

    if (shadow) return;

    // Normal
    M3DVector3f N; m3dSubtractVectors3(N, intersect_p, _pos); m3dNormalizeVector(N);

    // Light dir
    M3DVector3f L; m3dSubtractVectors3(L, light_pos, intersect_p); m3dNormalizeVector(L);
    float ndotl = std::max(0.0f, m3dDotProduct(N, L));

    // Diffuse
    for (int i = 0; i < 3; ++i) color[i] += kd * ndotl * light_col[i] * base[i];

    // View dir (from point to eye)
    M3DVector3f V; m3dCopyVector3(V, view); m3dScaleVector3(V, -1.0f); m3dNormalizeVector(V);

    // Reflection
    float twoNL = 2.0f * m3dDotProduct(N, L);
    M3DVector3f R; m3dLoadVector3(R, twoNL * N[0] - L[0], twoNL * N[1] - L[1], twoNL * N[2] - L[2]);
    m3dNormalizeVector(R);

    float rdotv = std::max(0.0f, m3dDotProduct(R, V));
    float spec = ks * powf(rdotv, shininess);
    for (int i = 0; i < 3; ++i) color[i] += spec * light_col[i];

    // Clamp
    for (int i = 0; i < 3; ++i) { if (color[i] < 0.0f) color[i] = 0.0f; if (color[i] > 1.0f) color[i] = 1.0f; }
}

// (Required by pure virtual in base — not used for local shading path)
void Sphere::get_reflect_direct(const M3DVector3f direct,
    const M3DVector3f intersect_p,
    M3DVector3f reflect_direct)
{
    // Simple perfect mirror reflection around normal
    M3DVector3f N; m3dSubtractVectors3(N, intersect_p, _pos); m3dNormalizeVector(N);
    float k = 2.0f * m3dDotProduct(direct, N);
    M3DVector3f term; m3dCopyVector3(term, N); m3dScaleVector3(term, k);
    m3dSubtractVectors3(reflect_direct, direct, term);
    m3dNormalizeVector(reflect_direct);
}

bool Sphere::get_refract_direct(const M3DVector3f, const M3DVector3f, M3DVector3f, float, bool)
{
    // Not used for local shading only; return false
    return false;
}
