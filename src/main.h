bool compare_floats(float x, float y);
bool compare_vector(glm::vec3 a, glm::vec3 b);

glm::vec3 diffuseOnly(const HitInfo hitInfo, const glm::vec3 lightPosition, glm::vec3 color, const bool interpolate);
glm::vec3 phongSpecularOnly(const HitInfo& hitInfo, const glm::vec3& lightPosition, const glm::vec3 lightColor,
    const glm::vec3& cameraPos, const bool interpolate);
bool visibleToLightTransparant(Ray ray, glm::vec3 lightPosition, HitInfo hitInfo, const BoundingVolumeHierarchy& bvh, int level, float& dim);
bool visibleToLight(Ray inComingRay, glm::vec3 lightPosition, HitInfo hitInfo, const BoundingVolumeHierarchy& bvh, int level, float& dim);
Ray computeRefractedRay(const BoundingVolumeHierarchy& bvh, const Ray& ray, const HitInfo& hitInfo, const bool interpolate);
Ray computeReflectedRay(const BoundingVolumeHierarchy& bvh, const Ray& ray, const HitInfo& hitInfo, const bool interpolate);
glm::vec3 samplePlanarLight(const HitInfo& hitInfo, const glm::vec3& lightPosition, const glm::vec3& lightColor,
    const BoundingVolumeHierarchy& bvh,
    const Ray& ray, const Scene& scene);
void drawPlainLight(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d);
glm::vec3 makePlainSamplePoints(int light_samples, const PlanarLight& planarLight, float width, float height,
    const Ray& rayToPlainCenter, HitInfo& hitInfo, const BoundingVolumeHierarchy& bvh, Ray& ray, const Scene& scene);
glm::vec3 sampleSphere(const HitInfo& hitInfo, const glm::vec3& lightPosition,
    const BoundingVolumeHierarchy& bvh,
    const Ray& ray, const Scene scene, const glm::vec3& lightColor);
glm::vec3 takeSphereSamples(Ray &randomRay, float distanceFromPlainCenterToSamplePoint, const glm::vec3 &samplePlainNormal,
const HitInfo &hitInfo, const BoundingVolumeHierarchy &bvh, const Ray &ray,
const glm::vec3 &lightColor, const Scene &scene);
glm::vec3 makeSphereSamplePoints(const int numberOfSamples, const SphericalLight& sphericalLight, const glm::vec3& p,
    const glm::vec3& n, const Ray& rayToSphereCenter,
    HitInfo& hitInfo, const BoundingVolumeHierarchy& bvh, Ray& ray, const Scene& scene);
void blur(Screen& screen, Screen& newScreen, const int filterSize);
Screen bloom(Screen& screen);
glm::vec3 pointLightShade(const Scene& scene, const BoundingVolumeHierarchy& bvh, const Ray& ray, const HitInfo& hitInfo, int level,const glm::vec3 &position,const glm::vec3 &color);
static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, int level);
Screen motionBlur(Screen& original, const Trackball& camera, const Scene& scene, const BoundingVolumeHierarchy& bvh,
    float strength, float smoothness);
