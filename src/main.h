bool compare_floats(float x, float y);

glm::vec3 diffuseOnly(const HitInfo hitInfo, const glm::vec3 lightPosition, glm::vec3 color, const bool interpolate);

glm::vec3 phongSpecularOnly(const HitInfo& hitInfo, const glm::vec3& lightPosition, const glm::vec3 lightColor,
    const glm::vec3& cameraPos, const bool interpolate);

bool visibleToLight(Ray inComingRay, glm::vec3 lightPosition, HitInfo hitInfo, const BoundingVolumeHierarchy& bvh);

glm::vec3 recursiveRay(const Ray& ray, const HitInfo& hitInfo, const BoundingVolumeHierarchy& bvh, int levels,
    const glm::vec3 lightPosition, const glm::vec3& lightColor, const glm::vec3& cameraPos, const bool interpolate);

glm::vec3 sampleSphere(const HitInfo& hitInfo, const glm::vec3& lightPosition, const glm::vec3& lightColor,
    const BoundingVolumeHierarchy& bvh,
    const Ray& ray, const bool interpolate);

glm::vec3 takeSamples(Ray& randomRay, float distanceFromPlainCenterToSamplePoint, const glm::vec3& samplePlainNormal,
    const HitInfo& hitInfo, const BoundingVolumeHierarchy& bvh, const Ray& ray, const glm::vec3& lightColor, const bool interpolate);

glm::vec3 makeSamplePoints(const int numberOfSamples, const SphericalLight& sphericalLight, const glm::vec3& p,
    const glm::vec3& n, const Ray& rayToSphereCenter,
    HitInfo& hitInfo, const BoundingVolumeHierarchy& bvh, Ray& ray, const bool interpolate);

static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray);


