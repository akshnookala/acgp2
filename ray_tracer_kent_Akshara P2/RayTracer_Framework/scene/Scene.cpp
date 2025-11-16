#include "Scene.h"
#include "../primitives/Wall.h"
#include "../primitives/Sphere.h"
#include "../common/math3d.h"

Scene::Scene()
{
    // Ambient light
    m3dLoadVector3(_am_light, 0.25f, 0.25f, 0.25f);

    // One white point light slightly above/front-left
    M3DVector3f Lpos; m3dLoadVector3(Lpos, 80.0f, 450.0f, 700.0f);
    M3DVector3f Lcol; m3dLoadVector3(Lcol, 1.0f, 1.0f, 1.0f);
    _sp_light.set_light(Lpos, Lcol);
}

Scene::~Scene()
{
    Basic_Primitive* prim = NULL;
    for (Prim_List::iterator it = _prim_list.begin(); it != _prim_list.end(); )
    {
        prim = *it;
        _prim_list.erase(it++);
        delete prim;
    }
}

void Scene::assemble()
{
    // Room corners
    M3DVector3f x0y0z0; m3dLoadVector3(x0y0z0, 0, 0, 0);
    M3DVector3f x1y0z0; m3dLoadVector3(x1y0z0, _dim[0], 0, 0);
    M3DVector3f x0y1z0; m3dLoadVector3(x0y1z0, 0, _dim[1], 0);
    M3DVector3f x1y1z0; m3dLoadVector3(x1y1z0, _dim[0], _dim[1], 0);
    M3DVector3f x0y0z1; m3dLoadVector3(x0y0z1, 0, 0, _dim[2]);
    M3DVector3f x1y0z1; m3dLoadVector3(x1y0z1, _dim[0], 0, _dim[2]);
    M3DVector3f x0y1z1; m3dLoadVector3(x0y1z1, 0, _dim[1], _dim[2]);
    M3DVector3f x1y1z1; m3dLoadVector3(x1y1z1, _dim[0], _dim[1], _dim[2]);

    // Updated Neon/Pastel Color Scheme (Back wall now Brown)
    M3DVector3f wall_color_left;   m3dLoadVector3(wall_color_left, 0.75f, 1.00f, 0.00f); // Butter Yellow
    M3DVector3f wall_color_right;  m3dLoadVector3(wall_color_right, 0.50f, 0.70f, 1.00f); // Sky Blue
    M3DVector3f wall_color_top;    m3dLoadVector3(wall_color_top, 0.80f, 0.58f, 0.98f); // Lavender
    M3DVector3f wall_color_bottom; m3dLoadVector3(wall_color_bottom, 0.18f, 0.18f, 0.18f); // Dark Grey
    M3DVector3f wall_color_back;   m3dLoadVector3(wall_color_back, 0.45f, 0.25f, 0.10f); // Brown (Updated)

    // Walls
    _prim_list.push_back(new Wall(x0y1z0, x0y1z1, x0y0z1, x0y0z0, wall_color_left));   // Left
    _prim_list.push_back(new Wall(x1y1z1, x1y1z0, x1y0z0, x1y0z1, wall_color_right));  // Right
    _prim_list.push_back(new Wall(x1y1z1, x0y1z1, x0y1z0, x1y1z0, wall_color_top));    // Top
    _prim_list.push_back(new Wall(x1y0z1, x1y0z0, x0y0z0, x0y0z1, wall_color_bottom)); // Bottom
    _prim_list.push_back(new Wall(x1y1z0, x0y1z0, x0y0z0, x1y0z0, wall_color_back));   // Back (BROWN)

    // Sphere #1 (Hot Pink)
    float rad1 = _dim[2] / 4.0f;
    M3DVector3f sp1_col; m3dLoadVector3(sp1_col, 1.00f, 0.41f, 0.71f);
    M3DVector3f sp1_pos; m3dLoadVector3(sp1_pos, _dim[0] - rad1 - 20.0f, rad1, _dim[2] * 2.0f / 3.0f - rad1);
    _prim_list.push_back(new Sphere(sp1_pos, rad1, sp1_col));

    // Sphere #2 (Lime)
    float rad2 = rad1 / 1.5f;
    M3DVector3f sp2_col; m3dLoadVector3(sp2_col, 0.75f, 1.00f, 0.00f);
    M3DVector3f sp2_pos; m3dLoadVector3(sp2_pos, rad2 + 20.0f, rad2, rad2 + 20.0f);
    _prim_list.push_back(new Sphere(sp2_pos, rad2, sp2_col));
}

Intersect_Cond Scene::intersection_check(const M3DVector3f start,
    const M3DVector3f dir,
    Basic_Primitive** prim_intersect,
    M3DVector3f closest_point)
{
    Basic_Primitive* prim = NULL;
    float distance = 0.0f;
    M3DVector3f point;
    float min_distance = 1e30f;
    *prim_intersect = NULL;

    Intersect_Cond ret = _k_miss;
    for (Prim_List::iterator it = _prim_list.begin(); it != _prim_list.end(); ++it)
    {
        prim = *it;
        Intersect_Cond tmp = prim->intersection_check(start, dir, distance, point);
        if (tmp != _k_miss && distance < min_distance)
        {
            min_distance = distance;
            *prim_intersect = prim;
            m3dCopyVector3(closest_point, point);
            ret = tmp;
        }
    }
    return ret;
}
