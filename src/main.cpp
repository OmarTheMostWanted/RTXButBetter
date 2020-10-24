#include "bounding_volume_hierarchy.h"
#include "disable_all_warnings.h"
#include "draw.h"
#include "image.h"
#include "ray_tracing.h"
#include "screen.h"
#include "trackball.h"
#include "window.h"
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
const int number_light_samples = 64; //set to 12 for faster rendering times
const int ray_tracing_levels = 8;


//debug ray colors:
//white = ray to point to intersection point if exists.
//blue = ray from intersection point to the objects that block the light.
//green = ray from the intersection point to the light if its not blocked.


bool compare_floats(float x, float y) {
    if (glm::abs(x - y) < 0.00001f)
        return true; //they are same
    return false; //they are not same
}

//Phong functions
glm::vec3
diffuseOnly(const HitInfo &hitInfo, const glm::vec3 &lightPosition, const glm::vec3 &lightColor) {
    auto cosAngle = glm::dot(hitInfo.normal, glm::normalize(lightPosition - hitInfo.intersectionPoint));
    if (cosAngle > 0) {
        auto res = hitInfo.material.kd * cosAngle;
        return res * lightColor;
    } else return glm::vec3(0);
}


glm::vec3 phongSpecularOnly(const HitInfo &hitInfo, const glm::vec3 &lightPosition, const glm::vec3 lightColor,
                            const glm::vec3 &cameraPos) {

    float cosNormalLight = glm::dot(glm::normalize(lightPosition - hitInfo.intersectionPoint), hitInfo.normal);

    if (cosNormalLight < 0) {
        return glm::vec3(0);
    }
    auto lightVec = glm::normalize(hitInfo.intersectionPoint - lightPosition);
    auto camVec = glm::normalize(cameraPos - hitInfo.intersectionPoint);
    auto normalN = glm::normalize(hitInfo.normal);
    auto reflectedLight = glm::normalize(lightVec - (2 * (glm::dot(lightVec, normalN))) * normalN);
    return hitInfo.material.ks *
           glm::pow(glm::max(glm::dot(reflectedLight, camVec), 0.0f), hitInfo.material.shininess) * lightColor;
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
bool visibleToLight(Ray inComingRay , glm::vec3 lightPosition, HitInfo hitInfo, const BoundingVolumeHierarchy &bvh) {

    Ray rayToLight = {hitInfo.intersectionPoint, glm::normalize(lightPosition - hitInfo.intersectionPoint) };
    auto cosLightNormal = glm::dot(rayToLight.direction , hitInfo.normal);
    //make sure that the ray hits the lit side of the mesh.
    if(cosLightNormal >= 0 ){

        //the ray is at the same side as the light relative to the normal.
        if(glm::dot(inComingRay.direction , hitInfo.normal) < 0  ){

            rayToLight.origin = rayToLight.origin + (rayToLight.direction * origin_shift);

            HitInfo hitInfo1;

            auto intersection = bvh.intersect(rayToLight, hitInfo1);

            float fromLightToPoint = glm::length(hitInfo.intersectionPoint - lightPosition);

            if (intersection) {

                float fromPointToIntersection = glm::length( hitInfo.intersectionPoint -  ( hitInfo.intersectionPoint + (rayToLight.direction * rayToLight.t)));

                //make sure if the there is an object closer to the point than the light ie the light is blocked
                if (fromPointToIntersection < fromLightToPoint) {
                    drawRay(rayToLight, glm::vec3(0 , 0 , 1));
                    return false;
                }
            }
            rayToLight.t = fromLightToPoint;
            drawRay(rayToLight, glm::vec3(0, 1.0f, 0.0f));
            return true;
        } else {
            return false;
        }
    } else {
        if(glm::dot(inComingRay.direction , hitInfo.normal) > 0  ){
            rayToLight.origin = rayToLight.origin + (rayToLight.direction * origin_shift);

            HitInfo hitInfo1;
            auto intersection = bvh.intersect(rayToLight, hitInfo1);

            float fromLightToPoint = glm::length(hitInfo.intersectionPoint - lightPosition);

            if (intersection) {

                float fromPointToIntersection = glm::length( hitInfo.intersectionPoint -  ( hitInfo.intersectionPoint + (rayToLight.direction * rayToLight.t)));

                if (fromPointToIntersection < fromLightToPoint) {
                    drawRay(rayToLight, glm::vec3(0 , 0 , 1));
                    return false;
                }
            }
            rayToLight.t = fromLightToPoint;
            drawRay(rayToLight, glm::vec3(0, 1.0f, 0.0f));
            return true;
        } else {
            return false;
        }
    }
}

//Recursive ray tracing
/**
 * Create recursive ray
 *
 * @param ray The ray that has a intersection
 * @param hitInfo
 * @return
 */
glm::vec3 recursiveRay(const Ray &ray, const HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, int levels,
                       const glm::vec3 lightPosition, const glm::vec3 &lightColor, const glm::vec3 &cameraPos) {
    if (hitInfo.material.ks == glm::vec3{0.0f}) return glm::vec3(0.0f);
    if (levels <= 0) return glm::vec3(0.0f);
    glm::vec3 normalizedN;
    if (glm::dot(ray.direction, hitInfo.normal) >= 0) {
        normalizedN = glm::normalize(hitInfo.normal);
    } else {
        normalizedN = -glm::normalize(hitInfo.normal);
    }
    glm::vec3 color = glm::vec3(0.0f);
    HitInfo hitInfoRecursive;

    glm::vec3 direction = glm::normalize(ray.direction - 2 * glm::dot(ray.direction, normalizedN) * normalizedN);
    Ray newRay = Ray{ hitInfo.intersectionPoint + 0.00001f * direction, direction };

    if (bvh.intersect(newRay, hitInfoRecursive)) {
        drawRay(newRay, glm::vec3(1, 0, 1));
        if (visibleToLight(newRay, lightPosition, hitInfoRecursive, bvh)) {
            color += recursiveRay(newRay, hitInfoRecursive, bvh, levels - 1, lightPosition, lightColor, cameraPos) +
                     phongSpecularOnly(hitInfoRecursive, lightPosition, lightColor, cameraPos) +
                     diffuseOnly(hitInfoRecursive, lightPosition, lightColor);

        }
    }
    return color;

}


glm::vec3 sampleSphere(const HitInfo &hitInfo, const glm::vec3 &lightPosition, const glm::vec3 &lightColor,
                       const BoundingVolumeHierarchy &bvh,
                       const Ray &ray) {

    glm::vec3 color = glm::vec3(0);
    Ray rayToLight = {hitInfo.intersectionPoint,
                      glm::normalize(lightPosition - hitInfo.intersectionPoint)};

    if (visibleToLight(ray, lightPosition, hitInfo, bvh)) {
        color += diffuseOnly(hitInfo, lightPosition, lightColor);
        color += phongSpecularOnly(hitInfo, lightPosition, lightColor, ray.origin);
    }
    color += recursiveRay(ray, hitInfo, bvh, ray_tracing_levels, lightPosition, lightColor, ray.origin);
    return color;
}


glm::vec3 takeSamples(Ray &randomRay, float distanceFromPlainCenterToSamplePoint, const glm::vec3 &samplePlainNormal,
                      const HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, const Ray &ray ,const glm::vec3& lightColor) {

    glm::vec3 color = glm::vec3(0.0f);

    glm::vec3 samplePoint1 = randomRay.origin + randomRay.direction * distanceFromPlainCenterToSamplePoint;
    glm::vec3 samplePoint2 = randomRay.origin - randomRay.direction * distanceFromPlainCenterToSamplePoint;

    glm::vec3 samplePoint3 = randomRay.origin + randomRay.direction * (distanceFromPlainCenterToSamplePoint/2);
    glm::vec3 samplePoint4 = randomRay.origin - randomRay.direction * (distanceFromPlainCenterToSamplePoint/2);

    randomRay.direction = glm::normalize(glm::cross(randomRay.direction, samplePlainNormal));

    glm::vec3 samplePoint5 = randomRay.origin + randomRay.direction * distanceFromPlainCenterToSamplePoint;
    glm::vec3 samplePoint6 = randomRay.origin - randomRay.direction * distanceFromPlainCenterToSamplePoint;

    glm::vec3 samplePoint7 = randomRay.origin + randomRay.direction * (distanceFromPlainCenterToSamplePoint/2);
    glm::vec3 samplePoint8 = randomRay.origin - randomRay.direction * (distanceFromPlainCenterToSamplePoint/2);

    color += sampleSphere(hitInfo, samplePoint1, lightColor, bvh, ray);
    color += sampleSphere(hitInfo, samplePoint2, lightColor, bvh, ray);
    color += sampleSphere(hitInfo, samplePoint3, lightColor, bvh, ray);
    color += sampleSphere(hitInfo, samplePoint4, lightColor, bvh, ray);

    color += sampleSphere(hitInfo, samplePoint5, lightColor, bvh, ray);
    color += sampleSphere(hitInfo, samplePoint6, lightColor, bvh, ray);
    color += sampleSphere(hitInfo, samplePoint7, lightColor, bvh, ray);
    color += sampleSphere(hitInfo, samplePoint8, lightColor, bvh, ray);

    return color;
}


glm::vec3 makeSamplePoints(const int numberOfSamples, const SphericalLight &sphericalLight, const glm::vec3 &p,
                           const glm::vec3 &n, const Ray &rayToSphereCenter,
                           HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, Ray &ray ) {
    //start by choosing a random point on the plane
//    float x = (float) rand() / RAND_MAX * 2 - 1;
//    float y = (float) rand() / RAND_MAX * 2 - 1;

    float x = 1;
    float y = 1;

    float z = ((-1 * n.x * x + n.x * p.x) + (-1 * n.y * y + n.y * p.y) + n.z * p.z) / (n.z);

    glm::vec3 color = glm::vec3 (0.0f);

    Ray randomRay;
    randomRay.origin = p;
    randomRay.direction = glm::normalize(glm::vec3{x, y, z} - p);

    float i = 0.0;
    int samplesTaken = 0;

    while(i < (glm::pi<float>()/4)){

        glm::mat4 rotation = glm::rotate( glm::mat4(1.0f) , i , randomRay.origin);

        randomRay.direction = glm::vec4 {randomRay.direction  , 1}* rotation;

        i += (glm::pi<float>() / ( (float) numberOfSamples) );

        float distanceFromPlainCenterToSamplePoint = (sphericalLight.radius * glm::length(p - rayToSphereCenter.origin)) /
                                                     glm::length(rayToSphereCenter.origin - sphericalLight.position);

        color += takeSamples(randomRay , distanceFromPlainCenterToSamplePoint , n , hitInfo , bvh , ray , sphericalLight.color);
        samplesTaken += 8;
    }
    return color;
}


// NOTE(Mathijs): separate function to make recursion easier (could also be done with lambda + std::function).
static glm::vec3 getFinalColor(const Scene &scene, const BoundingVolumeHierarchy &bvh, Ray ray) {

    HitInfo hitInfo;
    if (bvh.intersect(ray, hitInfo)) {

//         Draw a white debug ray.
        drawRay(ray, glm::vec3(1.0f));

        auto color = glm::vec3(0.0f);
        //shading
        // compute shading for each light source
        for (PointLight pointLight : scene.pointLights) {
            if (visibleToLight(ray, pointLight.position, hitInfo, bvh)) {
                color += diffuseOnly(hitInfo, pointLight.position, pointLight.color);
                color += phongSpecularOnly(hitInfo, pointLight.position, pointLight.color, ray.origin);
            }
            color += recursiveRay(ray, hitInfo, bvh, ray_tracing_levels, pointLight.position, pointLight.color, ray.origin);
        }

        for (SphericalLight sphericalLight : scene.sphericalLight) {
            Ray rayToSphereCenter = {hitInfo.intersectionPoint,
                                     glm::normalize(sphericalLight.position - hitInfo.intersectionPoint)};
            rayToSphereCenter.t = glm::length(sphericalLight.position - rayToSphereCenter.origin) -
                                  sphericalLight.radius;    //this is working so far.
            auto sampleLightPositionAtSphereCenter =
                    rayToSphereCenter.origin + rayToSphereCenter.t * rayToSphereCenter.direction;

            auto sphereLightSamples = sampleSphere(hitInfo, sampleLightPositionAtSphereCenter, sphericalLight.color,
                                                   bvh, ray);

            glm::vec3 samplePlainNormal = glm::normalize(sampleLightPositionAtSphereCenter - sphericalLight.position);

            sphereLightSamples += makeSamplePoints(number_light_samples/2, sphericalLight, sampleLightPositionAtSphereCenter,
                                                   samplePlainNormal, rayToSphereCenter,
                                                   hitInfo, bvh, ray);
            color += ( sphereLightSamples / (number_light_samples + 1.0f));

        }
        return color;
    } else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
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
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay));
        }
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
                    bvh.intersect(*optDebugRay, dummy);
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
                (void)getFinalColor(scene, bvh, *optDebugRay);
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
