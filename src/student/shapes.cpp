
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    Trace ret;
    ret.hit = false;

    float a = ray.dir.norm_squared();
    float b = 2.f * (dot(ray.point, ray.dir));
    float c = ray.point.norm_squared() - radius * radius;

    float delta = b * b - 4 * a * c;
    if(delta < 0) {
        return ret;
    }

    delta = sqrtf(delta);

    float t = (-b - delta) / a / 2.f;
    if(t < ray.dist_bounds.x || t > ray.dist_bounds.y) {
        t = (-b + delta) / a / 2.f;
        if(t < ray.dist_bounds.x || t > ray.dist_bounds.y) {
            return ret;
        }
    }

    ret.hit = true;                   // was there an intersection?
    ret.distance = t;                 // at what distance did the intersection occur?
    ret.position = ray.at(t);         // where was the intersection?
    ret.normal = ret.position.unit(); // what was the surface normal at the intersection?
    return ret;
}

} // namespace PT
