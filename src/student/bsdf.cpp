
#include "../rays/bsdf.h"
#include "../util/rand.h"

namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3{-dir.x, dir.y, -dir.z};
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    auto cos_out = out_dir.y;

    float ratio;
    if(cos_out > 0) {
        ratio = 1.f / index_of_refraction;
    } else {
        ratio = index_of_refraction;
    }

    auto sin_out = sqrt(1 - cos_out * cos_out);
    float sin_in = ratio * sin_out;

    if(sin_in >= 1) {
        was_internal = true;
        return {};
    }

    float cos_in = sqrt(1.f - sin_in * sin_in);

    float xz_ratio = sin_in / sin_out;

    was_internal = false;
    return Vec3{-out_dir.x * xz_ratio, cos_out > 0 ? -cos_in : cos_in, -out_dir.z * xz_ratio};
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.

    Scatter ret;
    ret.direction = sampler.sample();
    ret.attenuation = evaluate(out_dir, ret.direction);
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).

    return albedo * std::max(in_dir.y, 0.f);
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    return std::max(in_dir.y, 0.f) / PI_F;
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // (PathTracer): Task 5

    Scatter ret;
    ret.direction = reflect(out_dir);
    ret.attenuation = reflectance;
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?

    auto cos_out = fabs(out_dir.y);

    float n1 = 1.f;
    float n2 = index_of_refraction;

    float r0 = (n1 - n2) * (n1 - n2) / (n1 + n2) / (n1 + n2);
    float fresnel = r0 + (1 - r0) * powf(1 - cos_out, 5.f);

    auto do_reflect = RNG::coin_flip(fresnel);

    bool was_internal = false;

    Scatter ret;
    ret.direction =
        do_reflect ? reflect(out_dir) : refract(out_dir, index_of_refraction, was_internal);
    ret.attenuation = do_reflect ? reflectance : (was_internal ? Spectrum{} : transmittance);
    return ret;
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    bool was_internal;

    Scatter ret;
    ret.direction = refract(out_dir, index_of_refraction, was_internal);
    ret.attenuation = was_internal ? Spectrum{} : transmittance;
    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
