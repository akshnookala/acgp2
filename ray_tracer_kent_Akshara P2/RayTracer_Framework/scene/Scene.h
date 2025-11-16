#pragma once
#include "../common/common.h"
#include "../primitives/Basic_Primitive.h"
#include "Light.h"
#include <vector>

typedef std::vector<Basic_Primitive*> Prim_List;

class Scene
{
public:
    Scene();
    ~Scene();

    inline void set_dim(M3DVector3f dim) { m3dCopyVector3(_dim, dim); }
    void assemble();

    Intersect_Cond intersection_check(const M3DVector3f start,
        const M3DVector3f dir,
        Basic_Primitive** prim_intersect,
        M3DVector3f closest_point);

    const Light& get_sp_light() const { return _sp_light; }
    inline void get_amb_light(M3DVector3f am_light) const { m3dCopyVector3(am_light, _am_light); }

private:
    Prim_List   _prim_list;
    M3DVector3f _dim;
    Light       _sp_light;     // configured in constructor
    M3DVector3f _am_light;     // ambient color
};
