#pragma once
#include "ray_tracing.h"
#include "scene.h"

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

private:
    Scene* m_pScene;
    std::vector<Node> nodes; // contains all existing nodes
};

struct Node {

public:
    std::vector<Node> children; // two children nodes
    bool type; // 0 for interior, 1 for a leaf node
    AxisAlignedBox box; // coordinates of a axis-aligned box corespondign to the node
    std::vector<uint16_t> vertices; // indices of vertices contained in the node, empty for interior nodes

};
