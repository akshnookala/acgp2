#include "Triangle.h"
#include "../common/math3d.h"
#include <math.h>
#include <algorithm>

// Compute face normal (unnormalized)
void Triangle::normal(M3DVector3f n)
{
    M3DVector3f e1, e2;
    m3dSubtractVectors3(e1, _v1, _v0);
    m3dSubtractVectors3(e2, _v2, _v0);
    m3dCrossProduct(n, e1, e2);
    m3dNormalizeVector(n);
}

// Möller–Trumbore ray-triangle
Intersect_Cond Triangle::intersection_check(const M3DVector3f start, const M3DVector3f dir,
    float& distance, M3DVector3f intersection_p)
{
    const float EPS = 1e-6f;
    M3DVector3f e1, e2;
    m3dSubtractVectors3(e1, _v1, _v0);
    m3dSubtractVectors3(e2, _v2, _v0);

    M3DVector3f pvec; m3dCrossProduct(pvec, dir, e2);
    float det = m3dDotProduct(e1, pvec);
    if (fabs(det) < EPS) return _k_miss;

    float invDet = 1.0f / det;
    M3DVector3f tvec; m3dSubtractVectors3(tvec, start, _v0);
    float u = m3dDotProduct(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return _k_miss;

    M3DVector3f qvec; m3dCrossProduct(qvec, tvec, e1);
    float v = m3dDotProduct(dir, qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return _k_miss;

    distance = m3dDotProduct(e2, qvec) * invDet;
    if (distance < EPS) return _k_miss;

    M3DVector3f step; m3dCopyVector3(step, dir); m3dScaleVector3(step, distance);
    m3dAddVectors3(intersection_p, start, step);
    return _k_hit;
}

// Flat gray Phong shading (used for wall triangles if needed)
void Triangle::shade(M3DVector3f view,
    M3DVector3f intersect_p,
    const Light& sp_light,
    M3DVector3f am_light,
    M3DVector3f color,
    bool shadow)
{
    float ka = 0.2f, kd = 0.7f, ks = 0.3f, shininess = 12.0f;
    M3DVector3f base; m3dLoadVector3(base, 0.8f, 0.8f, 0.8f);

    for (int i = 0; i < 3; ++i) color[i] = ka * am_light[i] * base[i];
    if (shadow) return;

    M3DVector3f N; normal(N);

    M3DVector3f L, lpos, lcol; sp_light.get_light(lpos, lcol);
    m3dSubtractVectors3(L, lpos, intersect_p); m3dNormalizeVector(L);

    float ndotl = std::max(0.0f, m3dDotProduct(N, L));
    for (int i = 0; i < 3; ++i) color[i] += kd * ndotl * lcol[i] * base[i];

    M3DVector3f V; m3dCopyVector3(V, view); m3dScaleVector3(V, -1.0f); m3dNormalizeVector(V);

    float twoNL = 2.0f * m3dDotProduct(N, L);
    M3DVector3f R; m3dLoadVector3(R, twoNL * N[0] - L[0], twoNL * N[1] - L[1], twoNL * N[2] - L[2]);
    m3dNormalizeVector(R);

    float rdotv = std::max(0.0f, m3dDotProduct(R, V));
    float spec = ks * powf(rdotv, shininess);
    for (int i = 0; i < 3; ++i) color[i] += spec * lcol[i];

    for (int i = 0; i < 3; ++i) { if (color[i] < 0.0f) color[i] = 0.0f; if (color[i] > 1.0f) color[i] = 1.0f; }
}
