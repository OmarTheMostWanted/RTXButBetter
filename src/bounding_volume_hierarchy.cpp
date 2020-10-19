#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm\geometric.hpp>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, int numberOfSplits)
    : m_pScene(pScene), numberOfSplits(numberOfSplits)
{

    // as an example of how to iterate over all meshes in the scene, look at the intersect method below
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{

    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb { glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
}

int BoundingVolumeHierarchy::numLevels() const
{
    return 5;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                hitInfo.material = mesh.material;
hit = true;
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}

// creates new AxisAlignedBox containing all of the vertices and returns it
AxisAlignedBox BoundingVolumeHierarchy::createBoxFromVertices(std::vector<int> vertices)
{

    int minX = INT_MAX;
    int minY = INT_MAX;
    int minZ = INT_MAX;

    int maxX = INT_MIN;
    int maxY = INT_MIN;
    int maxZ = INT_MIN;

    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& vertex : mesh.vertices) {

            if (minX > vertex.p[0]) {

                minX = vertex.p[0];
            }
            if (maxX < vertex.p[0]) {

                maxX = vertex.p[0];
            }
            if (minY > vertex.p[1]) {

                minY = vertex.p[1];
            }
            if (maxY < vertex.p[1]) {

                maxY = vertex.p[2];
            }
            if (minZ > vertex.p[2]) {

                minZ = vertex.p[2];
            }
            if (maxZ < vertex.p[2]) {

                maxZ = vertex.p[2];
            }

        }
    }

    AxisAlignedBox newBox{ glm::vec3(minX, minY, minZ),  glm::vec3(maxX, maxY, maxZ) };

    return newBox;
}

// Creates a new node out of given vertices given as indices
Node BoundingVolumeHierarchy::createNodeFromVertices(std::vector<int> vertices)
{

    Node newNode;
    newNode.box = createBoxFromVertices(vertices);
    newNode.indices = vertices; // this will be replaces with the indices of children nodes if the node turns out to be an interior node
    newNode.type = 1; // a temporal status

    return newNode;
}

// Creates a new node out of given vertices given as indices and an already made box
Node BoundingVolumeHierarchy::createNodeFromVertices(std::vector<int> vertices, AxisAlignedBox box)
{

    Node newNode;
    newNode.box = box;
    newNode.indices = vertices; // this will be replaces with the indices of children nodes if the node turns out to be an interior node
    newNode.type = 1; // a temporal status

    return newNode;
}

// Divides the node using Surface Area Heuristics and sets the resulting nodes as its children
void BoundingVolumeHierarchy::splitNode(Node& node) {

    if (node.type == 1) {

        std::cout << "You cannot split a node that is already split" << std::endl;
    }
}

// Calculates the best splits along the X axis
void BoundingVolumeHierarchy::splitNodeX(Node& node) {

    int lowerX = node.box.upper[0];
    int higherX = node.box.lower[1];

    glm::vec3 normal = glm::vec3(1, 0, 0);

    int range = higherX - lowerX;
    int intreval = range / this->numberOfSplits;

    std::vector<int> firstGroupVertices;
    std::vector<int> secondGroupVertices;

    std::vector<Node> bestFitNodes;


    for (int i = 1; i < this->numberOfSplits; i++) {

        glm::vec3 point = glm::vec3(lowerX + intreval, 0, 0);

        for (const auto& mesh : m_pScene->meshes) {
            for (const auto& tri : mesh.triangles) {

                glm::vec3 controlVector0 = mesh.vertices[tri[0]].p - point;
                glm::vec3 controlVector1 = mesh.vertices[tri[1]].p - point;
                glm::vec3 controlVector2 = mesh.vertices[tri[2]].p - point;

                if (glm::dot(normal, controlVector0) >= 0 || glm::dot(normal, controlVector1) >= 0 || glm::dot(normal, controlVector2) >= 0) {

                    firstGroupVertices.push_back(tri[0]);
                    firstGroupVertices.push_back(tri[1]);
                    firstGroupVertices.push_back(tri[2]);
                }
                if (glm::dot(normal, controlVector0) < 0 || glm::dot(normal, controlVector1) < 0 || glm::dot(normal, controlVector2) < 0) {

                    secondGroupVertices.push_back(tri[0]);
                    secondGroupVertices.push_back(tri[1]);
                    secondGroupVertices.push_back(tri[2]);
                }
            }
        }

        AxisAlignedBox firstBox = createBoxFromVertices(firstGroupVertices);
        AxisAlignedBox secondBox = createBoxFromVertices(secondGroupVertices);

        float splitCost = calculateSplitCost(node.box, firstBox, secondBox);

        if (splitCost < node.splitCost) {

            node.splitCost = splitCost;
            Node firstChild = createNodeFromVertices(firstGroupVertices, firstBox);
            Node secondChild = createNodeFromVertices(secondGroupVertices, secondBox);
            
            this->nodes.push_back(firstChild);
            this->nodes.push_back(secondChild);

            // pushing the indices of newly created node to be our children nodes
            node.indices.push_back(this->nodes.size() - 2);
            node.indices.push_back(this->nodes.size() - 1);
            node.type = 0;
        }

        
    }
    
}

float BoundingVolumeHierarchy::calculateBoxVolume(AxisAlignedBox box) {

    return (box.upper[0] - box.lower[0]) * (box.upper[1] - box.lower[1]) * (box.upper[2] - box.lower[2]);
}

// Calculates the cost of a split
// the smaller the cost, the better the split is
float BoundingVolumeHierarchy::calculateSplitCost(AxisAlignedBox parentBox, AxisAlignedBox firstChild, AxisAlignedBox secondChild ) {

    return (calculateBoxVolume(firstChild) / calculateBoxVolume(parentBox)) + (calculateBoxVolume(secondChild) / calculateBoxVolume(parentBox));


}



