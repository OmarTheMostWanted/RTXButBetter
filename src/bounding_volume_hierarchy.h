#pragma once
#include "ray_tracing.h"
#include "scene.h"


struct Node;

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene, int numberOfSplits);

    int numberOfSplits; // number of splits being made for each AxisAlignedBox
    std::vector<Node> nodes;

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

    // Creates an AxisAlignedBoc out of given vertices, returns the created box
    AxisAlignedBox createBoxFromVertices(std::vector<int> vertices);
   // Creates a new node out of given vertices and adds it to the list of nodesub the scene, return the created node
    Node createNodeFromVertices(std::vector<int> vertices);
    Node createNodeFromVertices(std::vector<int> vertices, AxisAlignedBox box);

    // Splits the given axis so thath
    void splitNode(Node& node);

    void compareCostsAndUpdate(Node& parentNode, std::vector<std::vector<int>> dividedVertices);

    void splitNodeX(Node& node);
    void splitNodeY(Node& node);
    void splitNodeZ(Node& node);

    std::vector<std::vector<int>> divideByPlane(Node node, glm::vec3 normal, glm::vec3 point);

    float calculateBoxVolume(AxisAlignedBox box);

    float calculateSplitCost(AxisAlignedBox parentBox, AxisAlignedBox firstChild, AxisAlignedBox secondChild);
    
private:
    Scene* m_pScene;
};

struct Node {

public:
    std::vector<int> indices; // stores the indices of two children nodes in case of a interior node or the verices in case of a leaf ndoe
    bool type = 1; // 0 for interior, 1 for a leaf node
    AxisAlignedBox box; // coordinates of a axis-aligned box corespondign to the node
    float splitCost = FLT_MAX; // cost of split into two children nodes

};
