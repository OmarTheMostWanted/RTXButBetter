#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <iostream>
#include <glm\geometric.hpp>
#include <numeric>
#include <set>



struct Node;

const int SPLITS_PER_NODE = 2; // number of splits being made for each AxisAlignedBox
const int NUMBER_OF_LEVELS = 4;

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    std::vector<Node> nodes;

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    void drawNode(Node& node, int remainingLevels);
    int numLevels() const;

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

    // Creates an AxisAlignedBoc out of given triangles, returns the created box
    AxisAlignedBox createBoxFromTriangles(std::vector<int> triangles);
    Node& createParentNode();
    Node createNodeFromTriangles(std::vector<int> triangles);
    Node createNodeFromTriangles(std::vector<int> triangles, AxisAlignedBox& box);
   // Creates a new node out of given vertices and adds it to the list of nodesub the scene, return the created node

    // Splits the given node so that the split is the most profitable
    void splitNode(int nodeIndex, int remainingSplits);

    void compareCostsAndUpdate(int nodeIndex, std::vector<std::vector<int>> dividedVertices);

    std::vector<std::vector<int>> divideByPlane(int nodeIndex, glm::vec3 normal, glm::vec3 point);

    float calculateBoxVolume(AxisAlignedBox& box);

    float calculateSplitCost(AxisAlignedBox parentBox, AxisAlignedBox firstChild, AxisAlignedBox secondChild);

    std::set<int> retrieveVerticesIndicesFromTrianglesIndices(std::vector<int> trianglesIndices);
    
private:
    Scene* m_pScene;
};

struct Node {

public:
    std::vector<int> indices; // stores the indices of two children nodes in case of a interior node or the triangles in case of a leaf ndoe
    bool type = 1; // 0 for interior, 1 for a leaf node
    AxisAlignedBox box; // coordinates of a axis-aligned box corespondign to the node
    float splitCost = FLT_MAX; // cost of split into two children nodes

};
