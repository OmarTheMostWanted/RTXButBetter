#include "ray_tracing.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()

#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>

DISABLE_WARNINGS_POP()

#include <cmath>
#include <iostream>
#include <limits>

bool compare_float(float x, float y) {
    if (glm::abs(x - y) < 0.00001f)
        return true; //they are same
    return false; //they are not same
}

bool
pointInTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &n, const glm::vec3 &p) {
    auto v20 = v2 - v0;
    auto v10 = v1 - v0;
    auto vp0 = p - v0;

    //all the dots
    auto dot2020 = glm::dot(v20, v20);
    auto dot2010 = glm::dot(v20, v10);
    auto dot20p0 = glm::dot(v20, vp0);
    auto dot1010 = glm::dot(v10, v10);
    auto dot10p0 = glm::dot(v10, vp0);

    float d = 1 / (dot2020 * dot1010 - dot2010 * dot2010);

    //Barycentric coords u , v
    //P = v0 + u * (v2 - v0) + v * (v1 - v0)
    float u = (dot1010 * dot20p0 - dot2010 * dot10p0) * d;
    float v = (dot2020 * dot10p0 - dot2010 * dot20p0) * d;

    // Check if point is in triangle
    return (u >= 0) && (v >= 0) && (u + v < 1);
}

/*
for set of points P that are on the plane   (P - pointOnPlane) dot planeNormal = 0
 to find intersection point we need to find t such that    rayOrigin + t * rayDirection = P (a point on the plane).
 to find the t we substitute P in the plane equation as the rays equation (rayOrigin + t * rayDirection)
=> ((rayOrigin + t * rayDirection) - pointOnPlane) dot planeNormal = 0;
 some math magic => t = [ (pointOnPlane - rayOrigin)  dot planeNormal) / (rayDirection dot planeNormal) ]

 if (rayDirection dot planeNormal) == 0 we have to scenarios: => 1) if [(pointOnPlane - rayOrigin) dot planeNormal] == 0   then ray is inside the plane
                                                                  2) if [(pointOnPlane - rayOrigin) dot planeNormal] != 0   then the ray and plane do not intersect

 else we get t and the single intersection point is (rayOrigin + t * rayDirection)
*/
bool intersectRayWithPlane(const Plane &plane, Ray &ray) {
    glm::vec3 pointOnPlane = plane.D * plane.normal;

    if (compare_float(glm::dot(ray.direction, plane.normal), 0)) {

        if (compare_float(dot((pointOnPlane - ray.origin), plane.normal), 0)) {
            return true;
        } else return false;
    } else return true;
}

Plane trianglePlane(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2) {
    Plane plane;
    //compute plane normal
    glm::vec3 normal = glm::normalize(glm::cross((v0 - v2), (v1 - v2)));
    plane.normal = normal;

    //project any point from the plane on the normal
    float D = glm::dot(normal, v0);
    plane.D = D;

    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool
intersectRayWithTriangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, Ray &ray, HitInfo &hitInfo) {
    Plane plane = trianglePlane(v0, v1, v2);
    glm::vec3 pointOnPlane = v0;
    if (intersectRayWithPlane(plane, ray)) {
        //t = [ (pointOnPlane - rayOrigin)  dot planeNormal) / (rayDirection dot planeNormal) ]
        //intersection point is (rayOrigin + t * rayDirection)
        if (!compare_float(glm::dot(ray.direction, plane.normal), 0)) {
            float t = (glm::dot((pointOnPlane - ray.origin), plane.normal)) / glm::dot(ray.direction, plane.normal);
            glm::vec3 intersectionPoint = ray.origin + t * ray.direction;

            if (pointInTriangle(v0, v1, v2, plane.normal, intersectionPoint)) {

                if (t < 0) {
                    return false;
                }

                if (ray.t > t) {
                    ray.t = t;

                    hitInfo.intersectionPoint = intersectionPoint;
                    hitInfo.normal = plane.normal;
                    return true;

                }
            }
        } else {
            if (compare_float(dot((pointOnPlane - ray.origin), plane.normal), 0)) { //ray is on the plane


                ray.t = 0;

                hitInfo.intersectionPoint = ray.origin;
                hitInfo.normal = plane.normal;
                return true;
            }
        }
    } else return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere &sphere, Ray &ray, HitInfo &hitInfo) {
    //math magic from lec 9
    //shift to origin
    glm::vec3 RayOrigin = ray.origin - sphere.center;


    //pow() is slow
    float A = (ray.direction.x * ray.direction.x) + (ray.direction.y * ray.direction.y) +
              (ray.direction.z * ray.direction.z);
    float B = 2 * (ray.direction.x * RayOrigin.x + ray.direction.y * RayOrigin.y + ray.direction.z * RayOrigin.z);
    float C = (RayOrigin.x * RayOrigin.x) + (RayOrigin.y * RayOrigin.y) + (RayOrigin.z * RayOrigin.z) -
              (sphere.radius * sphere.radius);
    float D = B * B - (4 * A * C);


    if (D >= 0) {

        float t1 = ((-1 * B) + glm::sqrt(D)) / (2 * A);
        float t2 = ((-1 * B) - glm::sqrt(D)) / (2 * A);

        if (t1 >= 0 && t2 >= 0) {

            if (ray.t > t2) {
                ray.t = t2;
                return true;
            }

//            hitInfo.intersectionPoint = ray.origin + (ray.direction * tIn);
//            hitInfo.normal = glm::normalize(hitInfo.intersectionPoint - sphere.center);
            
        }

        if (t1 >= 0 && t2 < 0) { //t2 is the exit point
            if (ray.t > t1) {
                ray.t = t1;
                return true;
            }

//            hitInfo.intersectionPoint = ray.origin + (ray.direction * t2);
//            hitInfo.normal = glm::normalize(hitInfo.intersectionPoint - sphere.center);
            
        }

    }

    return false;

}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox &box, Ray &ray) {
    float tXMin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tYMin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tZMin = (box.lower.z - ray.origin.z) / ray.direction.z;

    float tXMax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float tYMax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float tZMax = (box.upper.z - ray.origin.z) / ray.direction.z;

    float tInX = glm::min(tXMin, tXMax);
    float tInY = glm::min(tYMin, tYMax);
    float tInZ = glm::min(tZMin, tZMax);

    float tOutX = glm::max(tXMin, tXMax);
    float tOutY = glm::max(tYMin, tYMax);
    float tOutZ = glm::max(tZMin, tZMax);

    float tInGlobal = glm::max(tInX, glm::max(tInY, tInZ));
    float tOutGlobal = glm::min(tOutX, glm::min(tOutY, tOutZ));

    if (tInGlobal > tOutGlobal || tOutGlobal < 0) return false; //ray does not cross or is in front of the box

    else {
        if (tInGlobal < 0) { //rays origin is in the box.
            if (ray.t > tOutGlobal) {
                ray.t = tOutGlobal;
            }
        } else if (tInGlobal < ray.t) //rays origin is behind the box. check if this box was closer.
            ray.t = tInGlobal;
        return true;
    }
}

