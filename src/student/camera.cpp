
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
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

    Vec3 sensor_point(screen_coord.x * sw, screen_coord.y * sh, -1.f);
    sensor_point *= focal_dist;
    sensor_point = iview * sensor_point;

    Samplers::Rect aperture_rect{Vec2(aperture)};
    Vec2 origin_xy = aperture_rect.sample() - aperture / 2.f;
    Vec3 origin = iview * Vec3(origin_xy.x, origin_xy.y, 0.f);

    return Ray(origin, sensor_point - origin);
}
