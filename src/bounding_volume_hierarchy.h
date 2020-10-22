#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <iostream>
#include <glm\geometric.hpp>
#include <numeric>
#include <set>



struct Node;

const int SPLITS_PER_NODE = 3; // number of splits being made for each AxisAlignedBox
const int NUMBER_OF_LEVELS = 10;

class BoundingVolumeHierarchy {
public:
    BoundingVolumeHierarchy(Scene* pScene);

    std::vector<Node> nodes;
    std::vector<Node> parentNodes;
    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    void drawNode(int NodeIndex, int remainingLevels);

    // Returns number of levels. The more levels the more nodes in the BVH.
    int numLevels() const;

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

    // Checks intersection with all nodes and returns true if any intersection occurs.
    // Calls intersectWithNodes function recursively.
    bool intersectBVH(Ray& ray, HitInfo& hitInfo);

    // Recursively checks if a ray intersects with nodes.
    // Return true if it does. Calls intersectWithTriangles if the nodes ia a leaf node.
    bool intersectWithNodes(int parentNodeIndex, int nodeIndex, Ray& ray, HitInfo& hitInfo);

    // Checks intersection of a ray with all the triangles contained in a leaf node.
    // Returns true if the intersection occurs.
    bool intersectWithTriangles(int parentNodeIndex, int nodeIndex, Ray& ray, HitInfo& hitInfo);
    // Creates an AxisAlignedBoc out of given triangles, returns the created box
    AxisAlignedBox createBoxFromTriangles(std::vector<int> triangles);

    // Create a parent node and add it to the main vector of vertices.
    Node& createParentNode(int meshNumber);
    // Creates a new node out of given vertices with given indices.
    Node createNodeFromTriangles(std::vector<int> triangles);
    // Creates a new node out of given vertices given as indices and an already made box.
    Node createNodeFromTriangles(std::vector<int> triangles, AxisAlignedBox& box);

    // Calculates the best splits of a node along all the 3 axes.
    // Each axis is being checked for SPLITS_PER_NODE - 1 different plane divisors.
    void splitNode(int nodeIndex, int remainingSplits);

    // Compares cost of performing a new split to the already existing one and updates it if the new cost turns out to be lower
    // Takes the parent node and a vector containing two groups of vertices' indices resulting from the division
    void compareCostsAndUpdate(int nodeIndex, std::vector<std::vector<int>> dividedVertices);

    // Replaces children of a node. Updates the content of the main nodes vector with the newly added nodes.
    void replaceChildren(int parentNodeIndex, Node& firstChild, Node& secondChild);

    // Divides the node along the given plane.
    // Plane is characterised by a normal and any point.
    // Returns vector of two vectors containing indicies of two groups of verticies resulting from the split.
    std::vector<std::vector<int>> divideByPlane(int nodeIndex, glm::vec3 normal, glm::vec3 point);

    // Calculates the volume of a given box.
    float calculateBoxVolume(AxisAlignedBox& box);

    // Calculates the cost of a split
    // the smaller the cost, the better the split is
    float calculateSplitCost(AxisAlignedBox parentBox, AxisAlignedBox firstChild, AxisAlignedBox secondChild);

    // Collects indices of all vertices that triangles with given indices contain and returns them as a set.
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
