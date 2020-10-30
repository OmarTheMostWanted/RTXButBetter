#include "bounding_volume_hierarchy.h"
#include "disable_all_warnings.h"
#include "draw.h"
#include "image.h"
#include "ray_tracing.h"
#include "screen.h"
#include "trackball.h"
#include "window.h"
#include "main.h"
// Disable compiler warnings in third-party code (which we cannot change).
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#ifdef USE_OPENMP
#include <omp.h>
#endif

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2 windowResolution{800, 800};
const std::filesystem::path dataPath{DATA_DIR};
const std::filesystem::path outputPath{OUTPUT_DIR};

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};


const float origin_shift = 0.0001f;
const int ray_tracing_levels = 8;
const bool interpolation_on = false;


const int number_sphere_light_samples = 16; //set to 12 for faster rendering times
const int number_plain_light_samples = 16;  // is set to the closest multiple of 8;


//Bloom Filter
const bool bloom_filter = false; // enable
const bool bloom_debug = false;
const float bloom_threshold = 0.9f;
const int bloom_filter_size = 8;

//Motion Blur
const bool motion_blur = false; // enable
const float motion_blur_strength = 0.01f; //how much the picture moves.
const float motion_blur_smoothness = 4.0f; // how many frames to per movement.
const bool motion_blur_horizontal = false;  //if false then the movement is vertical.
const bool custom_motion_blur_direction = false; // if this is set to true , then the motion blur direction is give by the vector below
const glm::vec3 motion_bur_direction = glm::vec3(2, 1, 0);

//debug ray colors:
//white = ray to point to intersection point if exists.
//blue = ray from intersection point to the objects that block the light.
//green = ray from the intersection point to the light if its not blocked.


bool compare_floats(float x, float y) {
    if (glm::abs(x - y) < 0.00001f)
        return true; //they are same
    return false; //they are not same
}

bool compare_vector(glm::vec3 a, glm::vec3 b) {
    if (compare_floats(a.x, b.x) && compare_floats(a.y, b.y) && compare_floats(a.z, b.z)) return true;
    else return false;
}

//Phong functions
glm::vec3
diffuseOnly(const HitInfo hitInfo, const glm::vec3 lightPosition, glm::vec3 color, const bool interpolate) {
    glm::vec3 normal;
    if (interpolate) normal = hitInfo.interpolatedNormal;
    else normal = hitInfo.normal;

    auto cosAngle = glm::dot(normal, glm::normalize(lightPosition - hitInfo.intersectionPoint));
    if (cosAngle > 0) {
        auto res = hitInfo.material.kd * glm::dot(normal, glm::normalize(lightPosition - hitInfo.intersectionPoint)) * color;
        return res;
    } else return glm::vec3(0);
}


glm::vec3 phongSpecularOnly(const HitInfo &hitInfo, const glm::vec3 &lightPosition, const glm::vec3 lightColor,
                            const glm::vec3 &cameraPos, const bool interpolate) {
    glm::vec3 normal;
    if (interpolate) normal = hitInfo.interpolatedNormal;
    else normal = hitInfo.normal;


    float cosNormalLight = glm::dot(glm::normalize(lightPosition - hitInfo.intersectionPoint), normal);

    if (cosNormalLight < 0) {
        return glm::vec3(0);
    }
    auto lightVec = glm::normalize(hitInfo.intersectionPoint - lightPosition);
    auto camVec = glm::normalize(cameraPos - hitInfo.intersectionPoint);
    auto normalN = glm::normalize(normal);
    auto reflectedLight = glm::normalize(lightVec - (2 * (glm::dot(lightVec, normalN))) * normalN);
    return hitInfo.material.ks *
           glm::pow(glm::max(glm::dot(reflectedLight, camVec), 0.0f), hitInfo.material.shininess) * lightColor;
}

bool visibleToLightTransparant(Ray ray, glm::vec3 lightPosition, HitInfo hitInfo, const BoundingVolumeHierarchy& bvh, int level, float &dim) {
    Ray newRay = { hitInfo.intersectionPoint + ray.direction * origin_shift, ray.direction };
    HitInfo newHit;
    dim = (dim + hitInfo.material.transparency) / 2;
    if (level > ray_tracing_levels) return true;
    if (bvh.intersect(newRay, newHit, 0, ray_tracing_levels) && compare_floats(newHit.material.transparency, 1.0f)) {
        float fromPointToIntersection = glm::length(hitInfo.intersectionPoint - (hitInfo.intersectionPoint + (newRay.direction * newRay.t)));
        float fromLightToPoint = glm::length(hitInfo.intersectionPoint - lightPosition);

        //make sure if the there is an object closer to the point than the light ie the light is blocked
        if (fromPointToIntersection < fromLightToPoint) {
            drawRay(newRay, glm::vec3(0, 0, 1));
            return false;
        }
        else {
            newRay.t = fromLightToPoint;
            drawRay(newRay, glm::vec3(0, 1.0f, 0.0f));
            return true;
        }
    }
    else return visibleToLightTransparant(newRay, lightPosition, newHit, bvh, level + 1, dim);
}

/**
 * Check if the point is visible from the light scours.
 *
 * @param ray From the point to the light
 * @param lightPosition
 * @param hitInfo
 * @param bvh
 * @return
 */
bool visibleToLight(Ray inComingRay , glm::vec3 lightPosition, HitInfo hitInfo, const BoundingVolumeHierarchy &bvh, int level, float &dim) {

    Ray rayToLight = {hitInfo.intersectionPoint, glm::normalize(lightPosition - hitInfo.intersectionPoint) };
    auto cosLightNormal = glm::dot(rayToLight.direction , hitInfo.normal);
    //make sure that the ray hits the lit side of the mesh.
    if(cosLightNormal >= 0 ){

        //the ray is at the same side as the light relative to the normal.
        if(glm::dot(inComingRay.direction , hitInfo.normal) < 0  ){

            rayToLight.origin = rayToLight.origin + (rayToLight.direction * origin_shift);

            HitInfo hitInfo1;

            auto intersection = bvh.intersect(rayToLight, hitInfo1, 0, ray_tracing_levels);

            float fromLightToPoint = glm::length(hitInfo.intersectionPoint - lightPosition);

            //if there is an intersection with a non transparent object
            if (intersection && compare_floats(hitInfo1.material.transparency, 1.0f)) {

                float fromPointToIntersection = glm::length( hitInfo.intersectionPoint -  ( hitInfo.intersectionPoint + (rayToLight.direction * rayToLight.t)));

                //make sure if the there is an object closer to the point than the light. ie: the light is blocked
                if (fromPointToIntersection < fromLightToPoint) {
                    drawRay(rayToLight, glm::vec3(0 , 0 , 1));
                    return false;
                }
            }
            else if (intersection) {
                drawRay(rayToLight, glm::vec3(0, 1.0f, 0.0f));
                dim = hitInfo.material.transparency;
                return visibleToLightTransparant(rayToLight, lightPosition, hitInfo1, bvh, 0, dim);
            }
            rayToLight.t = fromLightToPoint;
            drawRay(rayToLight, glm::vec3(0, 1.0f, 0.0f));
            return true;
        } else {
            return false; //if the light and the point we want to shade are on different sides of the mesh.  then the point is not visible to the light.
        }
    } else {
        if(glm::dot(inComingRay.direction , hitInfo.normal) > 0  ){
            rayToLight.origin = rayToLight.origin + (rayToLight.direction * origin_shift);

            HitInfo hitInfo1;
            auto intersection = bvh.intersect(rayToLight, hitInfo1, level, ray_tracing_levels);

            float fromLightToPoint = glm::length(hitInfo.intersectionPoint - lightPosition);

            if (intersection && compare_floats(hitInfo1.material.transparency, 1.0f)) {

                float fromPointToIntersection = glm::length(hitInfo.intersectionPoint - (hitInfo.intersectionPoint + (rayToLight.direction * rayToLight.t)));

                //make sure if the there is an object closer to the point than the light ie the light is blocked
                if (fromPointToIntersection < fromLightToPoint) {
                    drawRay(rayToLight, glm::vec3(0, 0, 1));
                    return false;
                }
            }
            else if(intersection) {
                rayToLight.t = fromLightToPoint;
                drawRay(rayToLight, glm::vec3(0, 1.0f, 0.0f));
                return visibleToLightTransparant(rayToLight, lightPosition, hitInfo1, bvh, 0, dim);
            }
            rayToLight.t = fromLightToPoint;
            drawRay(rayToLight, glm::vec3(0, 1.0f, 0.0f));
            return true;
        } else {
            return false;
        }
    }
}

//Refraction
Ray computeRefractedRay(const BoundingVolumeHierarchy& bvh, const Ray& ray, const HitInfo& hitInfo, const bool interpolate) {
    glm::vec3 dir_in = glm::normalize(ray.direction);
    glm::vec3 N;
    if (glm::dot(dir_in, hitInfo.normal) > 0) {
        N = glm::normalize(hitInfo.normal);
    }
    else {
        N = glm::normalize(-hitInfo.normal);
    }

    float ratio = ray.index_in / hitInfo.material.index_of_refraction;
    float c = glm::dot(N, dir_in);
    glm::vec3 dir_through = ratio * dir_in + ((ratio * c) - sqrtf(1 - powf(ratio, 2) * (1 - powf(c, 2)))) * N;
    return Ray{ hitInfo.intersectionPoint + 0.00001f * dir_through, dir_through , hitInfo.material.index_of_refraction };
}


//Recursive ray tracing
Ray computeReflectedRay(const BoundingVolumeHierarchy& bvh, const Ray& ray, const HitInfo& hitInfo, const bool interpolate) {
    glm::vec3 normalizedN;
    if (interpolate) normalizedN = glm::normalize(hitInfo.interpolatedNormal);
    else normalizedN = glm::normalize(hitInfo.normal);
    glm::vec3 dirNormal = glm::normalize(ray.direction);
    glm::vec3 direction = glm::normalize(dirNormal - 2 * glm::dot(dirNormal, normalizedN) * normalizedN);
    return Ray{ hitInfo.intersectionPoint + 0.00001f * direction, direction };
}

/**
 * Take light sample from the plain light.
 *
 * @param hitInfo For shading.
 * @param lightPosition Where to sample the plain light from.
 * @param lightColor  For shading.
 * @param bvh For shading and light visibility checks.
 * @param ray From the pixel to the point that needs to be shaded.
 * @param scene For shading.
 * @return The Color after sampling the light.
 */
glm::vec3 samplePlanarLight(const HitInfo &hitInfo, const glm::vec3 &lightPosition, const glm::vec3 &lightColor,
                            const BoundingVolumeHierarchy &bvh,
                            const Ray &ray, const Scene &scene) {
    glm::vec3 color = glm::vec3(0);
    Ray rayToLight = {hitInfo.intersectionPoint,
                      glm::normalize(lightPosition - hitInfo.intersectionPoint)};
    color += pointLightShade(scene, bvh, ray, hitInfo, 0, lightPosition, lightColor);
    return color;
}

/** Draw the Plain light, By default it is not visible in the Rasterization view.
 *  To see the plain lights, Draw debug rays. (press R on the mesh where the planar light is visible)
 *   a----------b
 *  |          |
 *  |          |
 *  c----------d
 * @param a
 * @param b
 * @param c
 * @param d
 */
void drawPlainLight(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d) {

    Ray ray1 = {a, glm::normalize(b - a), glm::length(a - b)};
    Ray ray2 = {b, glm::normalize(d - b), glm::length(d - b)};
    Ray ray3 = {d, glm::normalize(c - d), glm::length(c - d)};
    Ray ray4 = {c, glm::normalize(a - c), glm::length(c - a)};

    drawRay(ray1, glm::vec3(1.0f));
    drawRay(ray2, glm::vec3(1.0f));
    drawRay(ray3, glm::vec3(1.0f));
    drawRay(ray4, glm::vec3(1.0f));

}

/**
 * The plain light is sampled 8 times each stage.
 * The sample positions are from the figure below.
 * When the light is fully sampled,
 * 8 more samples are taken but with halve of the width and height of the previous size.
 * This keeps going untill the requiered number of samples is achieved.
 *
 * 3-------1-------2
 * |               |
 * 7       .       8
 * |               |
 * 6-------4-------5
 *
 * @param light_samples number of required samples.
 * @param planarLight The light.
 * @param width.
 * @param height.
 * @param rayToPlainCenter Center of the plain light.
 * @param hitInfo For shading.
 * @param bvh For shading.
 * @param ray For shading.
 * @param scene For shading.
 * @return The color of the point after sampling the plain light.
 */
glm::vec3 makePlainSamplePoints(int light_samples, const PlanarLight &planarLight, float width, float height,
                                const Ray &rayToPlainCenter,
                                HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, Ray &ray, const Scene &scene) {

    glm::vec3 color = glm::vec3(0.0f);


    glm::vec3 point1 = planarLight.position + planarLight.direction * (width / 2);
    glm::vec3 point2 = point1 + glm::cross(planarLight.direction, planarLight.normal) * (height / 2);
    glm::vec3 point3 = point1 - glm::cross(planarLight.direction, planarLight.normal) * (height / 2);

    glm::vec3 point4 = planarLight.position + planarLight.direction * (-width / 2);
    glm::vec3 point5 = point4 + glm::cross(planarLight.direction, planarLight.normal) * (height / 2);
    glm::vec3 point6 = point4 - glm::cross(planarLight.direction, planarLight.normal) * (height / 2);

    glm::vec3 point7 = planarLight.position + glm::cross(planarLight.direction, planarLight.normal) * (height / 2);
    glm::vec3 point8 = planarLight.position - glm::cross(planarLight.direction, planarLight.normal) * (height / 2);

    color += samplePlanarLight(hitInfo, point1, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point2, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point3, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point4, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point5, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point6, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point7, planarLight.color, bvh, ray, scene);
    color += samplePlanarLight(hitInfo, point8, planarLight.color, bvh, ray, scene);

    drawPlainLight(point3 , point2 , point6 , point5);

    color /= 8.0f; //average the color of all the light samples.

    if (light_samples >= 8) {
        auto newWidth = width - (width*(8.0f/(float)light_samples));
        auto newHeight = height - (height*(8.0f/(float)light_samples));
        color += makePlainSamplePoints(light_samples - 8, planarLight, newWidth, newHeight, rayToPlainCenter, hitInfo,
                                       bvh, ray, scene);
    }

    return color/2.0f;
}


/**
 * Sample the Sphere light from the given position.
 * @param hitInfo For Shading.
 * @param lightPosition Where to sample the light from.
 * @param bvh For Shading.
 * @param ray For Shading.
 * @param scene For Shading.
 * @param lightColor For Shading.
 * @return The color after sampling the light.
 */
glm::vec3 sampleSphere(const HitInfo &hitInfo, const glm::vec3 &lightPosition,
                       const BoundingVolumeHierarchy &bvh,
                       const Ray &ray, const Scene scene, const glm::vec3 &lightColor) {

    glm::vec3 color = glm::vec3(0);
    Ray rayToLight = {hitInfo.intersectionPoint,
                      glm::normalize(lightPosition - hitInfo.intersectionPoint)};
    color += pointLightShade(scene, bvh, ray, hitInfo, 0, lightPosition, lightColor);
    return color;
}


/**
 * Takes samples from a sphere , the number of sphere samples is defined by the global variable number_sphere_light_samples.
 * A direction ray is given , this ray is used to divided the sphere into four quads , two samples is taken from each quarter ;
 * One from the outs edge and one from the middle.
 *
 * @param randomRay this is used to sample 8 points on the sphere , with an angle of 90 between each to samples.
 * @param distanceFromPlainCenterToSamplePoint this is the center of the sampling plain.
 * @param samplePlainNormal sampling plain normal.
 * @param hitInfo For shading.
 * @param bvh For shading.
 * @param ray For shading.
 * @param lightColor For shading.
 * @param scene For shading.
 * @return the color of the point after sampling.
 */
glm::vec3
takeSphereSamples(Ray &randomRay, float distanceFromPlainCenterToSamplePoint, const glm::vec3 &samplePlainNormal,
                  const HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, const Ray &ray,
                  const glm::vec3 &lightColor, const Scene &scene) {


    glm::vec3 color = glm::vec3(0.0f);

    glm::vec3 samplePoint1 = randomRay.origin + randomRay.direction * distanceFromPlainCenterToSamplePoint;
    glm::vec3 samplePoint2 = randomRay.origin - randomRay.direction * distanceFromPlainCenterToSamplePoint;

    glm::vec3 samplePoint3 = randomRay.origin + randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);
    glm::vec3 samplePoint4 = randomRay.origin - randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);

    randomRay.direction = glm::normalize(glm::cross(randomRay.direction, samplePlainNormal));

    glm::vec3 samplePoint5 = randomRay.origin + randomRay.direction * distanceFromPlainCenterToSamplePoint;
    glm::vec3 samplePoint6 = randomRay.origin - randomRay.direction * distanceFromPlainCenterToSamplePoint;

    glm::vec3 samplePoint7 = randomRay.origin + randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);
    glm::vec3 samplePoint8 = randomRay.origin - randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);

    color += sampleSphere(hitInfo, samplePoint1, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint2, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint3, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint4, bvh, ray, scene, lightColor);

    color += sampleSphere(hitInfo, samplePoint5, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint6, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint7, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint8, bvh, ray, scene, lightColor);

    return color;
}


/**
 * This is where the 'Random Ray' that is used in the takeSphereSamples funtion is created.
 * To take samples from the sphere, An imaginary sampling plain is made.
 * This point is defined with a normal n and a point on the plain p.
 * n is in the ray to the point we want to shade starting from the sphere light center. p is the intersection point of n and the spheres surface.
 * Then the sphere is projected on the sample plain which creates a circle with the center p.
 * The radius of the circle is calculated using Thales's theorem.
 * The first quarter of the circle is divided equally according to the number of samples needed. (each of those divisions represents a 'Random Ray' used in the takeSphereSamples).
 *
 * @param numberOfSamples the required number of samples
 * @param sphericalLight the light
 * @param p the center of the sampling plain.
 * @param n the normal of the sampling plain.
 * @param rayToSphereCenter
 * @param hitInfo For Shading.
 * @param bvh For Shading.
 * @param ray For Shading.
 * @param scene For Shading.
 * @return the color after sampling the sphere.
 */
glm::vec3 makeSphereSamplePoints(const int numberOfSamples, const SphericalLight &sphericalLight, const glm::vec3 &p,
                                 const glm::vec3 &n, const Ray &rayToSphereCenter,
                                 HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, Ray &ray , const Scene &scene) {
    float x = 1;
    float y = 1;

    float z = ((-1 * n.x * x + n.x * p.x) + (-1 * n.y * y + n.y * p.y) + n.z * p.z) / (n.z);

    glm::vec3 color = glm::vec3(0.0f);

    Ray randomRay;
    randomRay.origin = p;
    randomRay.direction = glm::normalize(glm::vec3{x, y, z} - p);

    float i = 0.0;
    int samplesTaken = 0;

    //rotate the random ray according to the number of samples needed.  (8 samples are taken at each oriantation)
    while (i < (glm::pi<float>() / 4)) {
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), i, randomRay.origin);
        randomRay.direction = glm::vec4{randomRay.direction, 1} * rotation;
        i += (glm::pi<float>() / ((float) numberOfSamples));
        float distanceFromPlainCenterToSamplePoint =
                (sphericalLight.radius * glm::length(p - rayToSphereCenter.origin)) /
                glm::length(rayToSphereCenter.origin - sphericalLight.position);
        color += takeSphereSamples(randomRay, distanceFromPlainCenterToSamplePoint, n, hitInfo, bvh, ray,
                                   sphericalLight.color, scene);
        samplesTaken += 8;
    }
    return color;
}

/**
 * Blur images by averaging pixels.
 * Using the Box filter from the lectures.
 *
 * @param screen to take the original image from.
 * @param newScreen store the result of the blurred image
 * @param filterSize
 */
void blur(Screen &screen, Screen &newScreen, const int filterSize) {
    auto avg = glm::vec3(0.0f);

    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x < windowResolution.x; x++) {
            for (int a = -filterSize; a < filterSize + 1; ++a) {
                for (int b = -filterSize; b < filterSize + 1; ++b) {
                    avg += newScreen.getPixel(x + a, y + b);
                }
            }
            avg /= (2 * filterSize + 1) * (2 * filterSize + 1);
//            screen.setPixel(x, y, (avg + screen.getPixel(x, y))/2.0f );  //take the average instead of adding the two components , to avoid having color values more than 1
            screen.setPixel(x, y, (avg + screen.getPixel(x, y)) );
            avg = glm::vec3(0.0f);
        }
    }
}

/**
 * This is used to only keep the pixels that are brighter than a given threshold for each color component.
 *
 * @param screen the input to threshold.
 * @return the screen with only the pixels that are higher than thr threshold.
 */
Screen bloom(Screen &screen) {

    Screen newScreen(screen.m_resolution);
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x < windowResolution.x; x++) {
            const int i = (screen.m_resolution.y - 1 - y) * screen.m_resolution.x + x;
            glm::vec3 colorComponents(0.0f);

            if (screen.m_textureData[i].x > bloom_threshold) {
                colorComponents.x = screen.m_textureData[i].x;
            }

            if (screen.m_textureData[i].y > bloom_threshold) {
                colorComponents.y = screen.m_textureData[i].y;
            }

            if (screen.m_textureData[i].z > bloom_threshold) {
                colorComponents.z = screen.m_textureData[i].z;
            }

            newScreen.setPixel(x, y, colorComponents);
        }
    }

    return newScreen;

}



glm::vec3 pointLightShade(const Scene& scene, const BoundingVolumeHierarchy& bvh,const Ray &ray, const HitInfo &hitInfo, int level, const glm::vec3 &position, const glm::vec3 &lightcolor) {
    // compute shading for each light source
    glm::vec3 color = glm::vec3(0.0f);
    float dim = 1.0f;
    if (visibleToLight(ray, position, hitInfo, bvh, level, dim)) {
        color += dim * diffuseOnly(hitInfo, position, lightcolor, interpolation_on);
        //std::cout << color.x << "  " << color.y << "   " << color.z << std::endl;
        color += dim * phongSpecularOnly(hitInfo, position, lightcolor, ray.origin, interpolation_on);
        //std::cout << color.x << "  " << color.y << "   " << color.z << std::endl;
    }
    //recursive ray tracing
    Ray reflected_ray = computeReflectedRay(bvh, ray, hitInfo, interpolation_on);
    if (!compare_vector(hitInfo.material.ks, glm::vec3(0.0f))) {
        glm::vec3 reflected_color = hitInfo.material.ks * getFinalColor(scene, bvh, reflected_ray, level + 1);
        color += reflected_color;
    }
    //std::cout << color.x << "  " << color.y << "   " << color.z << std::endl;
    //refraction and transparancy
    Ray refracted_ray = computeRefractedRay(bvh, ray, hitInfo, interpolation_on);
    if (!compare_floats(hitInfo.material.transparency, 1.0f)) {
        glm::vec3 refracted_color = hitInfo.material.transparency * getFinalColor(scene, bvh, refracted_ray, level + 1);
        color += refracted_color;
    }
    //std::cout << color.x << "  " << color.y << "   " << color.z << std::endl;
    return color;
}




// NOTE(Mathijs): separate function to make recursion easier (could also be done with lambda + std::function).
/**
 * If there is an intersection, shad the intersection point according to all the light sourse in the scene.
 *
 * @param scene has all the light sources.
 * @param bvh  used to calculate the intersections.
 * @param ray The ray from the pixel to the scene
 * @param level to keep track of the levels of ray tracing
 * @return the finial color of a pixel (before appling bloom or motion blur).
 */
static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, int level) {
    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo, level, ray_tracing_levels)) { //if there is an intersection shade the intersection point

        //         Draw a white debug ray.
        drawRay(ray, glm::vec3(1.0f));
        auto color = glm::vec3(0.0f);

        //shading
        for (PointLight pointLight : scene.pointLights) {
            color += pointLightShade(scene, bvh, ray, hitInfo, level, pointLight.position, pointLight.color);
        }

       for (SphericalLight sphericalLight : scene.sphericalLight) {

                /*
                 * sample the spheres by creating an imaginary sampling plain.
                 */

                Ray rayToSphereCenter = { hitInfo.intersectionPoint,
                                         glm::normalize(sphericalLight.position - hitInfo.intersectionPoint) };
                rayToSphereCenter.t = glm::length(sphericalLight.position - rayToSphereCenter.origin) -
                    sphericalLight.radius;  //this is working so far.
                auto sampleLightPositionAtSphereCenter =
                    rayToSphereCenter.origin + rayToSphereCenter.t * rayToSphereCenter.direction;

                auto sphereLightSamples = sampleSphere(hitInfo, sampleLightPositionAtSphereCenter,
                    bvh, ray, scene, sphericalLight.color);

                glm::vec3 samplePlainNormal = glm::normalize(sampleLightPositionAtSphereCenter - sphericalLight.position);

                sphereLightSamples += makeSphereSamplePoints(number_sphere_light_samples / 2, sphericalLight,
                    sampleLightPositionAtSphereCenter,
                    samplePlainNormal, rayToSphereCenter,
                    hitInfo, bvh, ray, scene);
                color += (sphereLightSamples / (number_sphere_light_samples + 1.0f));
            }
            for (PlanarLight planarLight : scene.planarLights) {

                Ray rayToPlainCenter = { hitInfo.intersectionPoint,
                                        glm::normalize(planarLight.position - hitInfo.intersectionPoint) };

                rayToPlainCenter.t = glm::length(planarLight.position - rayToPlainCenter.origin);

                glm::vec3 sampleLightPositionAtPlainCenter =
                    rayToPlainCenter.origin + rayToPlainCenter.t * rayToPlainCenter.direction;

                auto planarLightSamples = samplePlanarLight(hitInfo, sampleLightPositionAtPlainCenter, planarLight.color,
                    bvh, ray, scene);  //take sample from the center

                planarLightSamples += makePlainSamplePoints(number_plain_light_samples, planarLight, planarLight.width,
                    planarLight.height, rayToPlainCenter, hitInfo, bvh, ray, scene);

                color += planarLightSamples / 2.0f;
        }
            if (color.x > 1.0f) color.x = 1.0f;
            if (color.y > 1.0f) color.y = 1.0f;
            if (color.z > 1.0f) color.z = 1.0f;

            return color;
    }
        else {
            // Draw a red debug ray if the ray missed.
            drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
            // Set the color of the pixel to black if the ray misses.
            return glm::vec3(0.0f);
        }
    }

/**
 * to simulate movement, render the image multiple times from different camera angles.
 * then average the resulting image
 *
 * @param original the original image to apply motion to.
 * @param camera the camera.
 * @param scene For shading.
 * @param bvh For shading.
 * @param strength how much should the objects shift. (the strength of the motion)
 * @param smoothness how many how many frames to render while shifting (smoothness)
 * @return the new motion blurred image.
 */
Screen motionBlur(Screen &original, const Trackball &camera, const Scene &scene, const BoundingVolumeHierarchy &bvh,
                  float strength, float smoothness) {

    float steps = strength / smoothness;

    float shift = steps;

    Screen motionBlur(original.m_resolution);

    int counter = 0;

    while (shift <= strength) {

        std::cout << "Motion Frame " << counter << std::endl;

        Screen screenMoved(original.m_resolution);
        for (int y = 0; y < windowResolution.y; y++) {
            for (int x = 0; x != windowResolution.x; x++) {
                // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
                const glm::vec2 normalizedPixelPos{
                        float(x) / windowResolution.x * 2.0f - 1.0f,
                        float(y) / windowResolution.y * 2.0f - 1.0f
                };
                Ray cameraRay = camera.generateRay(normalizedPixelPos);

                if (motion_blur_horizontal && !custom_motion_blur_direction) {
                    cameraRay.origin.x = cameraRay.origin.x + shift;
                } else if (!motion_blur_horizontal && !custom_motion_blur_direction) {
                    cameraRay.origin.y += shift;
                } else if (custom_motion_blur_direction) {
                    cameraRay.origin = cameraRay.origin + (glm::normalize(motion_bur_direction)) * shift;
                }


                screenMoved.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, 0));
                motionBlur.setPixel(x, y, (original.getPixel(x, y) + screenMoved.getPixel(x, y)) / 2.0f);
            }
        }


        counter++;
        shift += steps;
    }

//    for (int y = 0; y < windowResolution.y; y++) {
//        for (int x = 0; x != windowResolution.x; x++) {
//            motionBlur.setPixel(x , y , motionBlur.getPixel(x , y) / (float)counter  );
//        }
//    }

    return motionBlur;

}

static void setOpenGLMatrices(const Trackball &camera);

static void renderOpenGL(const Scene &scene, const Trackball &camera, int selectedLight);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void
renderRayTracing(const Scene &scene, const Trackball &camera, const BoundingVolumeHierarchy &bvh, Screen &screen) {
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos{
                    float(x) / windowResolution.x * 2.0f - 1.0f,
                    float(y) / windowResolution.y * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, 0));
        }
    }

    if (bloom_filter) {
        std::cout << "Blooming img" << std::endl;
        Screen newScreen = bloom(screen);
        if (bloom_debug) screen.m_textureData = newScreen.m_textureData;
        blur(screen, newScreen, bloom_filter_size);
    }

    if (motion_blur) {
        std::cout << "applying motion blur with a shift by " << motion_blur_strength << " in " << motion_blur_smoothness
                  << " steps." << std::endl;
        screen.m_textureData = motionBlur(screen, camera, scene, bvh, motion_blur_strength,
                                          motion_blur_smoothness).m_textureData;
    }
}

int main(int argc, char **argv) {

    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
              << std::endl;

    Window window{"Final Project - Part 2", windowResolution, OpenGLVersion::GL2};
    Screen screen{windowResolution};
    Trackball camera{&window, glm::radians(50.0f), 3.0f};
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType{SceneType::SingleTriangle};
    std::optional<Ray> optDebugRay;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh{&scene};

    int bvhDebugLevel = 0;
    bool debugBVH{false};
    ViewMode viewMode{ViewMode::Rasterization};

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_R: {
                    // Shoot a ray. Produce a ray from camera to the far plane.
                    const auto tmp = window.getNormalizedCursorPos();
                    optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
                    viewMode = ViewMode::Rasterization;
                }
                    break;
                case GLFW_KEY_ESCAPE: {
                    window.close();
                }
                    break;
            };
        }
    });

    int selectedLight { 0 };
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project - Part 2");
        {
            constexpr std::array items { "SingleTriangle", "Cube", "Cornell Box (with mirror)", "Cornell Box (spherical light and mirror)", "Monkey", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                bvh = BoundingVolumeHierarchy(&scene);
                if (optDebugRay) {
                    HitInfo dummy {};
                    bvh.intersect(*optDebugRay, dummy, 0, ray_tracing_levels);
                }
            }
        }
        {
            constexpr std::array items { "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            {
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                renderRayTracing(scene, camera, bvh, screen);
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
            }
            screen.writeBitmapToFile(outputPath / "render.bmp");
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH)
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
            {
                std::vector<std::string> options;
                for (size_t i = 0; i < scene.pointLights.size(); i++) {
                    options.push_back("Point Light " + std::to_string(i + 1));
                }
                for (size_t i = 0; i < scene.sphericalLight.size(); i++) {
                    options.push_back("Spherical Light " + std::to_string(i + 1));
                }

                std::vector<const char*> optionsPointers;
                std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers),
                    [](const auto& str) { return str.c_str(); });

                ImGui::Combo("Selected light", &selectedLight, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            }

            {
                const auto showLightOptions = [](auto& light) {
                    ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                    ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                    if constexpr (std::is_same_v<std::decay_t<decltype(light)>, SphericalLight>) {
                        ImGui::DragFloat("Light radius", &light.radius, 0.01f, 0.01f, 0.5f);
                    }
                };
                if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                    // Draw a big yellow sphere and then the small light sphere on top.
                    showLightOptions(scene.pointLights[selectedLight]);
                } else {
                    // Draw a big yellow sphere and then the smaller light sphere on top.
                    showLightOptions(scene.sphericalLight[selectedLight - scene.pointLights.size()]);
                }
            }
        }

        if (ImGui::Button("Add point light")) {
            scene.pointLights.push_back(PointLight { glm::vec3(0.0f), glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() - 1);
        }
        if (ImGui::Button("Add spherical light")) {
            scene.sphericalLight.push_back(SphericalLight { glm::vec3(0.0f), 0.1f, glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() + scene.sphericalLight.size() - 1);
        }
        if (ImGui::Button("Remove selected light")) {
            if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                scene.pointLights.erase(std::begin(scene.pointLights) + selectedLight);
            } else {
                scene.sphericalLight.erase(std::begin(scene.sphericalLight) + (selectedLight - scene.pointLights.size()));
            }
            selectedLight = 0;
        }

        // Clear screen.
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            renderOpenGL(scene, camera, selectedLight);
            if (optDebugRay) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;
                (void)getFinalColor(scene, bvh, *optDebugRay, 0);
                enableDrawRay = false;
            }
            glPopAttrib();
        } break;
        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            bvh.debugDraw(bvhDebugLevel);
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0; // execution never reaches this point
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);
    setOpenGLMatrices(camera);

    glDisable(GL_LIGHTING);
    // Render point lights as very small dots
    for (const auto& light : scene.pointLights)
        drawSphere(light.position, 0.01f, light.color);
    for (const auto& light : scene.sphericalLight)
        drawSphere(light.position, light.radius, light.color);

    if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
        if (selectedLight < static_cast<int>(scene.pointLights.size())) {
            // Draw a big yellow sphere and then the small light sphere on top.
            const auto& light = scene.pointLights[selectedLight];
            drawSphere(light.position, 0.05f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, 0.01f, light.color);
            glEnable(GL_DEPTH_TEST);
        } else {
            // Draw a big yellow sphere and then the smaller light sphere on top.
            const auto& light = scene.sphericalLight[selectedLight - scene.pointLights.size()];
            drawSphere(light.position, light.radius + 0.01f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, light.radius, light.color);
            glEnable(GL_DEPTH_TEST);
        }
    }

    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    int i = 0;
    const auto enableLight = [&](const auto& light) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4 { light.position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4 { glm::clamp(light.color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4 { 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.pointLights)
        enableLight(light);
    for (const auto& light : scene.sphericalLight)
        enableLight(light);

    // Draw the scene and the ray (if any).
    drawScene(scene);

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

glm::vec3
takeSphereSamples(Ray &randomRay, float distanceFromPlainCenterToSamplePoint, const glm::vec3 &samplePlainNormal,
                  const HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, const Ray &ray,
                  const glm::vec3 &lightColor, const Scene &scene) {


    glm::vec3 color = glm::vec3(0.0f);

    glm::vec3 samplePoint1 = randomRay.origin + randomRay.direction * distanceFromPlainCenterToSamplePoint;
    glm::vec3 samplePoint2 = randomRay.origin - randomRay.direction * distanceFromPlainCenterToSamplePoint;

    glm::vec3 samplePoint3 = randomRay.origin + randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);
    glm::vec3 samplePoint4 = randomRay.origin - randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);

    randomRay.direction = glm::normalize(glm::cross(randomRay.direction, samplePlainNormal));

    glm::vec3 samplePoint5 = randomRay.origin + randomRay.direction * distanceFromPlainCenterToSamplePoint;
    glm::vec3 samplePoint6 = randomRay.origin - randomRay.direction * distanceFromPlainCenterToSamplePoint;

    glm::vec3 samplePoint7 = randomRay.origin + randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);
    glm::vec3 samplePoint8 = randomRay.origin - randomRay.direction * (distanceFromPlainCenterToSamplePoint / 2);

    color += sampleSphere(hitInfo, samplePoint1, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint2, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint3, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint4, bvh, ray, scene, lightColor);

    color += sampleSphere(hitInfo, samplePoint5, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint6, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint7, bvh, ray, scene, lightColor);
    color += sampleSphere(hitInfo, samplePoint8, bvh, ray, scene, lightColor);

    return color;
}
