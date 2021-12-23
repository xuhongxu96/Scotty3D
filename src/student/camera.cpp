
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    const float pi = atanf(1.f) * 4.f;

    float sh = tan(vert_fov * pi / 180.f / 2.f) * 2.f;
    float sw = aspect_ratio * sh;

    Vec4 dir(screen_coord.x * sw, screen_coord.y * sh, -1.f, 0.f);
    dir = iview * dir;
    return Ray(position, dir.xyz());
}
