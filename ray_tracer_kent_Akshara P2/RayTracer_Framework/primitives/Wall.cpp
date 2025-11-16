#include "Wall.h"
#include "Triangle.h"
#include "../common/math3d.h"
#include <math.h>
#include <algorithm>

void Wall::load_texture(std::string) {
    // Texture support not used in this project
}

void Wall::texture_color(M3DVector3f, M3DVector3f color) {
    // For now, fallback to solid color (texture optional)
    m3dCopyVector3(color, _color);
}

void Wall::get_texel(float, float, M3DVector3f color) {
    // Texture not used, so apply flat color
    m3dCopyVector3(color, _color);
}

// Intersection: check both triangles and pick nearest
Intersect_Cond Wall::intersection_check(const M3DVector3f start, const M3DVector3f dir,
    float& distance, M3DVector3f intersection_p)
{
    float d1 = 0.0f, d2 = 0.0f;
    M3DVector3f p1, p2;

    Intersect_Cond r1 = _tr1.intersection_check(start, dir, d1, p1);
    Intersect_Cond r2 = _tr2.intersection_check(start, dir, d2, p2);

    if (r1 == _k_miss && r2 == _k_miss) return _k_miss;

    if (r1 != _k_miss && (r2 == _k_miss || d1 < d2)) {
        distance = d1;
        m3dCopyVector3(intersection_p, p1);
        return r1;
    }
    else {
        distance = d2;
        m3dCopyVector3(intersection_p, p2);
        return r2;
    }
}

// Local Phong shading
void Wall::shade(M3DVector3f view,
    M3DVector3f intersect_p,
    const Light& sp_light,
    M3DVector3f am_light,
    M3DVector3f color,
    bool shadow)
{
    float ka = _ka, kd = _kd, ks = _ks;
    const float shininess = 10.0f;

    // Base color from wall
    M3DVector3f base;
    get_color(intersect_p, base);

    // Ambient
    for (int i = 0; i < 3; ++i)
        color[i] = ka * am_light[i] * base[i];

    if (shadow) return;

    // Normal from triangle 1 (shared plane)
    M3DVector3f N;
    _tr1.normal(N);

    // Light direction
    M3DVector3f L, lpos, lcol;
    sp_light.get_light(lpos, lcol);
    m3dSubtractVectors3(L, lpos, intersect_p);
    m3dNormalizeVector(L);

    float ndotl = std::max(0.0f, m3dDotProduct(N, L));

    // Diffuse
    for (int i = 0; i < 3; ++i)
        color[i] += kd * ndotl * lcol[i] * base[i];

    // View direction
    M3DVector3f V;
    m3dCopyVector3(V, view);
    m3dScaleVector3(V, -1.0f);
    m3dNormalizeVector(V);

    // Reflection vector
    float twoNL = 2.0f * m3dDotProduct(N, L);
    M3DVector3f R;
    m3dLoadVector3(R, twoNL * N[0] - L[0], twoNL * N[1] - L[1], twoNL * N[2] - L[2]);
    m3dNormalizeVector(R);

    float rdotv = std::max(0.0f, m3dDotProduct(R, V));
    float spec = ks * powf(rdotv, shininess);

    for (int i = 0; i < 3; ++i)
        color[i] += spec * lcol[i];

    // Clamp final color
    for (int i = 0; i < 3; ++i) {
        if (color[i] < 0.0f) color[i] = 0.0f;
        if (color[i] > 1.0f) color[i] = 1.0f;
    }
}

void Wall::get_reflect_direct(const M3DVector3f direct,
    const M3DVector3f intersect_p,
    M3DVector3f reflect_direct)
{
    // Reflect off wall normal
    M3DVector3f N;
    _tr1.normal(N);

    float k = 2.0f * m3dDotProduct(direct, N);
    M3DVector3f term;
    m3dCopyVector3(term, N);
    m3dScaleVector3(term, k);

    m3dSubtractVectors3(reflect_direct, direct, term);
    m3dNormalizeVector(reflect_direct);
}
