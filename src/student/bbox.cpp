
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    auto tmin = times.x;
    auto tmax = times.y;

    auto txmin = (min.x - ray.point.x) / ray.dir.x;
    auto txmax = (max.x - ray.point.x) / ray.dir.x;
    if(txmin > txmax) std::swap(txmin, txmax);

    if(tmin > txmax || txmin > tmax) return false;

    if(txmin > tmin) tmin = txmin;
    if(tmax > txmax) tmax = txmax;

    auto tymin = (min.y - ray.point.y) / ray.dir.y;
    auto tymax = (max.y - ray.point.y) / ray.dir.y;
    if(tymin > tymax) std::swap(tymin, tymax);

    if(tmin > tymax || tymin > tmax) return false;

    if(tymin > tmin) tmin = tymin;
    if(tmax > tymax) tmax = tymax;

    auto tzmin = (min.z - ray.point.z) / ray.dir.z;
    auto tzmax = (max.z - ray.point.z) / ray.dir.z;
    if(tzmin > tzmax) std::swap(tzmin, tzmax);

    if(tmin > tzmax || tzmin > tmax) return false;

    if(tzmin > tmin) tmin = tzmin;
    if(tmax > tzmax) tmax = tzmax;

    times.x = tmin;
    times.y = tmax;

    return true;
}
