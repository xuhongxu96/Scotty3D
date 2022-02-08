
#include "../rays/env_light.h"

#include <limits>

namespace PT {

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    return uniform_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // Second, swap to image_sampler.pdf().
    return 1 / (4.0f * PI_F);
}

Spectrum Env_Map::evaluate(Vec3 dir) const {

    // (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.

    auto cos_t = fabs(dir.y);
    auto sin_t = sqrtf(1 - cos_t * cos_t);
    auto t = acosf(cos_t);

    auto cos_p = std::min(1.f, fabs(dir.x / sin_t));
    auto sin_p = dir.z /*/ sin_t*/;
    auto p = acosf(cos_p);
    if(sin_p < 0) {
        p = 2.f * PI_F - p;
    }

    auto x = (image.dimension().first - 2) * p / 2.f / PI_F;
    auto y = (image.dimension().second - 2) * t / PI_F;

    int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
    auto s00 = image.at(x0, y0);
    auto s01 = image.at(x0, y0 + 1);
    auto s10 = image.at(x0 + 1, y0);
    auto s11 = image.at(x0 + 1, y0 + 1);

    return ((x - x0) * s10 + (x0 + 1 - x) * s00) * (y0 + 1 - y) +
           ((x - x0) * s11 + (x0 + 1 - x) * s01) * (y - y0);
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
