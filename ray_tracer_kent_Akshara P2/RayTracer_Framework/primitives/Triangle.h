#pragma once
#include "Basic_Primitive.h"

class Triangle : public Basic_Primitive
{
public:
    Triangle(M3DVector3f v0, M3DVector3f v1, M3DVector3f v2)
        : Basic_Primitive(_k_triangle)
    {
        m3dCopyVector3(_v0, v0);
        m3dCopyVector3(_v1, v1);
        m3dCopyVector3(_v2, v2);
    }
    ~Triangle() {}

    inline void get_vertex(M3DVector3f v0, M3DVector3f v1, M3DVector3f v2) const
    {
        m3dCopyVector3(v0, _v0); m3dCopyVector3(v1, _v1); m3dCopyVector3(v2, _v2);
    }

    Intersect_Cond intersection_check(const M3DVector3f start, const M3DVector3f dir,
        float& distance, M3DVector3f intersection_p);
    void normal(M3DVector3f n);

    // Local Phong shade for a flat triangle (gray)
    void shade(M3DVector3f view, M3DVector3f intersect_p, const Light& sp_light,
        M3DVector3f am_light, M3DVector3f color, bool shadow);

    void get_reflect_direct(const M3DVector3f, const M3DVector3f, M3DVector3f) {}

    void get_properties(float& ks, float& kt, float& ws, float& wt) const
    {
        ks = 0.2f; kt = 0.0f; ws = 0.0f; wt = 0.0f;
    }

private:
    M3DVector3f _v0, _v1, _v2;
};
