#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm\geometric.hpp>
#include <numeric>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, int numberOfSplits)
    : m_pScene(pScene), numberOfSplits(numberOfSplits)
{

    // for now do it only for one mesh
    Node currentNode = createParentNode();

    splitNode(currentNode, numberOfSplits);
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
    return numberOfSplits;
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

// creates new AxisAlignedBox containing all of the vertices with given indices and returns it
AxisAlignedBox BoundingVolumeHierarchy::createBoxFromVertices(std::vector<int> indices)
{

    int minX = INT_MAX;
    int minY = INT_MAX;
    int minZ = INT_MAX;

    int maxX = INT_MIN;
    int maxY = INT_MIN;
    int maxZ = INT_MIN;

    const std::vector<Vertex> vertices = m_pScene->meshes[0].vertices;

    for (int i : indices) {

        Vertex vertex = vertices[i];

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

    return AxisAlignedBox { glm::vec3(minX, minY, minZ),  glm::vec3(maxX, maxY, maxZ) };
}

Node BoundingVolumeHierarchy::createParentNode() {

    int minX = INT_MAX;
    int minY = INT_MAX;
    int minZ = INT_MAX;

    int maxX = INT_MIN;
    int maxY = INT_MIN;
    int maxZ = INT_MIN;

    int numberOfIndices = 0;

    for (const auto& mesh : m_pScene->meshes) {

        numberOfIndices += mesh.vertices.size();
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

    std::vector<int> allIndices(numberOfIndices);
    std::iota(begin(allIndices), end(allIndices), 0);

    return Node {allIndices, 1, newBox, FLT_MAX};
}

// Creates a new node out of given vertices with given indices
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

// Calculates the best splits along all the 3 axes
void BoundingVolumeHierarchy::splitNode(Node& node, int remainingSplits) {

    int lowerX = node.box.lower[0];
    int higherX = node.box.upper[0];

    int lowerY = node.box.lower[1];
    int higherY = node.box.upper[1];

    int lowerZ = node.box.lower[2];
    int higherZ = node.box.upper[2];

    int rangeX = higherX - lowerX;
    int rangeY = higherY - lowerY;
    int rangeZ = higherZ - lowerZ;

    int intervalX = rangeX / this->numberOfSplits;
    int intervalY = rangeY / this->numberOfSplits;
    int intervalZ = rangeZ / this->numberOfSplits;

    glm::vec3 normalX = glm::vec3(1, 0, 0);
    glm::vec3 normalY = glm::vec3(0, 1, 0);
    glm::vec3 normalZ = glm::vec3(0, 0, 1);

    for (int i = 1; i < this->numberOfSplits; i++) {

        glm::vec3 pointX = glm::vec3(lowerX + i * intervalX, 0, 0);
        glm::vec3 pointY = glm::vec3(0, lowerY + i * intervalY, 0);
        glm::vec3 pointZ = glm::vec3(0, 0, lowerZ + i * intervalZ);

        std::vector<std::vector<int>> resultingVerticesX = divideByPlane(node, normalX, pointX);
        std::vector<std::vector<int>> resultingVerticesY = divideByPlane(node, normalY, pointX);
        std::vector<std::vector<int>> resultingVerticesZ = divideByPlane(node, normalZ, pointX);

        compareCostsAndUpdate(node, resultingVerticesX);
        compareCostsAndUpdate(node, resultingVerticesY);
        compareCostsAndUpdate(node, resultingVerticesZ);
    }

    remainingSplits--;

    if (remainingSplits > 0) {

        splitNode(this->nodes[node.indices[0]], remainingSplits - 1);
        splitNode(this->nodes[node.indices[1]], remainingSplits - 1);
    }
    
}

// Compares cost of performing a new split to the already existing one and updates it if the new cost turns out to be lower
// Takes the parent node and a vector containing two groups of vertices' indices resulting from the division
void BoundingVolumeHierarchy::compareCostsAndUpdate(Node& parentNode, std::vector<std::vector<int>> dividedVertices) {

    AxisAlignedBox firstBox = createBoxFromVertices(dividedVertices[0]);
    AxisAlignedBox secondBox = createBoxFromVertices(dividedVertices[1]);

    float newCost = calculateSplitCost(parentNode.box, firstBox, secondBox);

    if (newCost < parentNode.splitCost) {

        parentNode.splitCost = newCost;
        Node firstChild = createNodeFromVertices(dividedVertices[0], firstBox);
        Node secondChild = createNodeFromVertices(dividedVertices[1], secondBox);

        this->nodes.push_back(firstChild);
        this->nodes.push_back(secondChild);

        // pushing the indices of newly created node to be our children nodes
        parentNode.indices.clear();
        parentNode.indices.push_back(this->nodes.size() - 2);
        parentNode.indices.push_back(this->nodes.size() - 1);
        parentNode.type = 0;
    }
}

// Divides the node along the given plane
// Plane is characterised by a normal and any point
// Returns vector of two vectors containing indicies of two groups of verticies resulting from the split
std::vector<std::vector<int>> BoundingVolumeHierarchy::divideByPlane(Node node, glm::vec3 normal, glm::vec3 point) {

    std::vector<int> firstGroupVertices;
    std::vector<int> secondGroupVertices;

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

    std::vector<std::vector<int>> result;
    result.push_back(firstGroupVertices);
    result.push_back(secondGroupVertices);

    return result;

}

// Calculates the volume of the given box
float BoundingVolumeHierarchy::calculateBoxVolume(AxisAlignedBox box) {

    return (box.upper[0] - box.lower[0]) * (box.upper[1] - box.lower[1]) * (box.upper[2] - box.lower[2]);
}

// Calculates the cost of a split
// the smaller the cost, the better the split is
float BoundingVolumeHierarchy::calculateSplitCost(AxisAlignedBox parentBox, AxisAlignedBox firstChild, AxisAlignedBox secondChild ) {

    return (calculateBoxVolume(firstChild) / calculateBoxVolume(parentBox)) + (calculateBoxVolume(secondChild) / calculateBoxVolume(parentBox));


}



