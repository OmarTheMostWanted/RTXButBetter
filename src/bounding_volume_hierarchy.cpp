#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <glm\geometric.hpp>
#include <numeric>
#include <set>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{

    // for now do it only for one mesh
    for (int i = 0; i < pScene->meshes.size(); i++) {

        createParentNode(i);

        splitNode(i, NUMBER_OF_LEVELS);

    }
    
    // as an example of how to iterate over all meshes in the scene, look at the intersect method below
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // level 0 is the parent node

    for (int i = 0; i < parentNodes.size(); i++) {

        drawNode(i, level);
    }
    
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    //AxisAlignedBox aabb { glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    //drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
}

void BoundingVolumeHierarchy::drawNode(int nodeIndex, int remainingLevels) {

    
    Node& node = this->nodes[nodeIndex];
    if (remainingLevels == 0) {

        drawAABB(node.box, DrawMode::Wireframe, glm::vec3(0.0f, 1.0f, 0.0f), 1.0);
        return;
    }

    if (node.type == 0) {

        for (int i : node.indices) {

            drawNode(i, remainingLevels - 1);
        }
    }
    
}

int BoundingVolumeHierarchy::numLevels() const
{
    return NUMBER_OF_LEVELS;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;

    for (int i = 0; i < parentNodes.size(); i++) {


    }
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

bool BoundingVolumeHierarchy::intersectWithNode(int nodeIndex, Ray& ray, HitInfo& hitInfo) const {

    return false;
}


// creates new AxisAlignedBox containing all of the vertices with given indices and returns it
AxisAlignedBox BoundingVolumeHierarchy::createBoxFromTriangles(std::vector<int> triangles)
{

    if (triangles.empty()) {

        return AxisAlignedBox{ glm::vec3(0),  glm::vec3(0) };
    }
    float minX = INT_MAX;
    float minY = INT_MAX;
    float minZ = INT_MAX;

    float maxX = INT_MIN;
    float maxY = INT_MIN;
    float maxZ = INT_MIN;

    const std::vector<Vertex> vertices = m_pScene->meshes[0].vertices;

    std::set<int> verticesIndices = retrieveVerticesIndicesFromTrianglesIndices(triangles);

    

    for (int vertexIndex : verticesIndices) {

        if (minX > vertices[vertexIndex].p[0]) {

            minX = vertices[vertexIndex].p[0];
        }
        if (maxX < vertices[vertexIndex].p[0]) {

            maxX = vertices[vertexIndex].p[0];
        }
        if (minY > vertices[vertexIndex].p[1]) {

            minY = vertices[vertexIndex].p[1];
        }
        if (maxY < vertices[vertexIndex].p[1]) {

            maxY = vertices[vertexIndex].p[1];
        }
        if (minZ > vertices[vertexIndex].p[2]) {

            minZ = vertices[vertexIndex].p[2];
        }
        if (maxZ < vertices[vertexIndex].p[2]) {

            maxZ = vertices[vertexIndex].p[2];
        }

    }
    return AxisAlignedBox { glm::vec3(minX, minY, minZ),  glm::vec3(maxX, maxY, maxZ) };
}

// Create a parent node and add it to the main vector of vertices
Node& BoundingVolumeHierarchy::createParentNode(int meshNumber) {

    float minX = INT_MAX;
    float minY = INT_MAX;
    float minZ = INT_MAX;

    float maxX = INT_MIN;
    float maxY = INT_MIN;
    float maxZ = INT_MIN;

    Mesh mesh = this->m_pScene->meshes[meshNumber];

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

                maxY = vertex.p[1];
            }
            if (minZ > vertex.p[2]) {

                minZ = vertex.p[2];
            }
            if (maxZ < vertex.p[2]) {

                maxZ = vertex.p[2];
            }
        
    }

    AxisAlignedBox newBox;
    newBox.lower = glm::vec3(minX, minY, minZ);
    newBox.upper = glm::vec3(maxX, maxY, maxZ);

    std::vector<int> allIndices(m_pScene->meshes[0].triangles.size());
    std::iota(begin(allIndices), end(allIndices), 0);

    Node parentNode{ allIndices, 1, newBox, FLT_MAX };
    nodes.push_back(parentNode);
    parentNodes.push_back(parentNode);

    return parentNode;
}

// Creates a new node out of given vertices with given indices
Node BoundingVolumeHierarchy::createNodeFromTriangles(std::vector<int> triangles)
{
    Node newNode;
    newNode.box = createBoxFromTriangles(triangles);
    newNode.indices = triangles; // this will be replaces with the indices of children nodes if the node turns out to be an interior node
    newNode.type = 1; // a temporal status
    newNode.splitCost = FLT_MAX;

    return newNode;
}

// Creates a new node out of given vertices given as indices and an already made box
Node BoundingVolumeHierarchy::createNodeFromTriangles(std::vector<int> triangles, AxisAlignedBox& box)
{

    Node newNode;
    newNode.box = box;
    newNode.indices = triangles; // this will be replaces with the indices of children nodes if the node turns out to be an interior node
    newNode.type = 1; // a temporal status
    newNode.splitCost = FLT_MAX;

    return newNode;
}

// Calculates the best splits along all the 3 axes
void BoundingVolumeHierarchy::splitNode(int nodeIndex, int remainingSplits) {


    Node& node = nodes[nodeIndex];
    if (calculateBoxVolume(node.box) == 0) {

        return;
    }


    float lowerX = node.box.lower[0];
    float higherX = node.box.upper[0];

    float lowerY = node.box.lower[1];
    float higherY = node.box.upper[1];

    float lowerZ = node.box.lower[2];
    float higherZ = node.box.upper[2];

    float rangeX = higherX - lowerX;
    float rangeY = higherY - lowerY;
    float rangeZ = higherZ - lowerZ;

    float intervalX = rangeX / (float) SPLITS_PER_NODE;
    float intervalY = rangeY / (float) SPLITS_PER_NODE;
    float intervalZ = rangeZ / (float) SPLITS_PER_NODE;

    glm::vec3 normalX = glm::vec3(1, 0, 0);
    glm::vec3 normalY = glm::vec3(0, 1, 0);
    glm::vec3 normalZ = glm::vec3(0, 0, 1);

    // Creating vectors to hold resulting triangles after each split
    std::vector<std::vector<std::vector<int>>> resultingTrianglesX(SPLITS_PER_NODE - 1);
    std::vector<std::vector<std::vector<int>>> resultingTrianglesY(SPLITS_PER_NODE - 1);
    std::vector<std::vector<std::vector<int>>> resultingTrianglesZ(SPLITS_PER_NODE - 1);

    for (int i = 1; i < SPLITS_PER_NODE; i++) {

        glm::vec3 pointX = glm::vec3(lowerX + i * intervalX, 0, 0);
        glm::vec3 pointY = glm::vec3(0, lowerY + i * intervalY, 0);
        glm::vec3 pointZ = glm::vec3(0, 0, lowerZ + i * intervalZ);

        resultingTrianglesX[i - 1] = divideByPlane(nodeIndex, normalX, pointX);
        resultingTrianglesY[i - 1] = divideByPlane(nodeIndex, normalY, pointY);
        resultingTrianglesZ[i - 1] = divideByPlane(nodeIndex, normalZ, pointZ);

        
    }

    for (int i = 0; i < SPLITS_PER_NODE - 1; i++) {

        compareCostsAndUpdate(nodeIndex, resultingTrianglesX[i]);
        compareCostsAndUpdate(nodeIndex, resultingTrianglesY[i]);
        compareCostsAndUpdate(nodeIndex, resultingTrianglesZ[i]);
    }

    Node& parentNode = nodes[nodeIndex];
    Node& firstChild = nodes[nodes[nodeIndex].indices[0]];
    Node& secondChild = nodes[nodes[nodeIndex].indices[0]];

    if (nodes[nodeIndex].box.lower == firstChild.box.lower && nodes[nodeIndex].box.upper == firstChild.box.upper) {

        return;
    }
    
    if (nodes[nodeIndex].box.lower == secondChild.box.lower && nodes[nodeIndex].box.upper == secondChild.box.upper) {

        return;
    }

    remainingSplits--;

    if (remainingSplits >= 0) {

        splitNode(nodes[nodeIndex].indices[0], remainingSplits);
        splitNode(nodes[nodeIndex].indices[1], remainingSplits);
    }
    
}

// Compares cost of performing a new split to the already existing one and updates it if the new cost turns out to be lower
// Takes the parent node and a vector containing two groups of vertices' indices resulting from the division
void BoundingVolumeHierarchy::compareCostsAndUpdate(int nodeIndex, std::vector<std::vector<int>> dividedTriangles) {

    Node& parentNode = nodes[nodeIndex];
    AxisAlignedBox firstBox = createBoxFromTriangles(dividedTriangles[0]);
    AxisAlignedBox secondBox = createBoxFromTriangles(dividedTriangles[1]);

    float newCost = calculateSplitCost(parentNode.box, firstBox, secondBox);

    if (newCost < parentNode.splitCost) {

        parentNode.splitCost = newCost;
        Node firstChild = createNodeFromTriangles(dividedTriangles[0], firstBox);
        Node secondChild = createNodeFromTriangles(dividedTriangles[1], secondBox);

        replaceChildren(nodeIndex, firstChild, secondChild);
        
    }
}

void BoundingVolumeHierarchy::replaceChildren(int parentNodeIndex, Node& firstChild, Node& secondChild) {

    Node* parentNode = &this->nodes[parentNodeIndex];
    
    if (parentNode->type == 0) {

        this->nodes.pop_back();
        this->nodes.pop_back();
    }

    this->nodes.push_back(firstChild);
    this->nodes.push_back(secondChild);

    // pushing the indices of newly created node to be our children nodes
    parentNode = &this->nodes[parentNodeIndex];

    parentNode->indices.clear();
    parentNode->indices.push_back(this->nodes.size() - 2);
    parentNode->indices.push_back(this->nodes.size() - 1);
    parentNode->type = 0;

}

// Divides the node along the given plane
// Plane is characterised by a normal and any point
// Returns vector of two vectors containing indicies of two groups of verticies resulting from the split
std::vector<std::vector<int>> BoundingVolumeHierarchy::divideByPlane(int nodeIndex, glm::vec3 normal, glm::vec3 point) {

    Node* node = &nodes[nodeIndex];
    std::vector<int> firstGroupTriangles;
    std::vector<int> secondGroupTriangles;

    Mesh mesh = m_pScene->meshes[0];
    std::vector<glm::uvec3> triangles = m_pScene->meshes[0].triangles;
    std::vector<Vertex> vertices = m_pScene->meshes[0].vertices;

    for (int triangleIndex: node->indices){

        glm::uvec3 triangle = triangles[triangleIndex];
           glm::vec3 controlVector0 = vertices[triangle[0]].p - point;
         glm::vec3 controlVector1 = vertices[triangle[1]].p - point;
            glm::vec3 controlVector2 = vertices[triangle[2]].p - point;

            /*bool check1 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector1) >= 0;
            bool check2 = glm::dot(normal, controlVector1) >= 0 && glm::dot(normal, controlVector2) >= 0;
            bool check3 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector2) >= 0;*/


            if (glm::dot(normal, controlVector0) >= 0 || glm::dot(normal, controlVector1) >= 0 || glm::dot(normal, controlVector2) >= 0) {

                firstGroupTriangles.push_back(triangleIndex);
            }

            else {

                secondGroupTriangles.push_back(triangleIndex);
            }
        
    }

    std::vector<std::vector<int>> result;
    result.push_back(firstGroupTriangles);
    result.push_back(secondGroupTriangles);

    return result;

}

// Calculates the volume of the given box
float BoundingVolumeHierarchy::calculateBoxVolume(AxisAlignedBox& box) {

    return (box.upper[0] - box.lower[0]) * (box.upper[1] - box.lower[1]) * (box.upper[2] - box.lower[2]);
}

// Calculates the cost of a split
// the smaller the cost, the better the split is
float BoundingVolumeHierarchy::calculateSplitCost(AxisAlignedBox parentBox, AxisAlignedBox firstChild, AxisAlignedBox secondChild ) {

    float firstChildVolume = calculateBoxVolume(firstChild);
    float secondChildVolume = calculateBoxVolume(secondChild);
    float parentNodeVolume = calculateBoxVolume(parentBox);

    float cost = (firstChildVolume / parentNodeVolume) + (secondChildVolume / parentNodeVolume);

   // if (firstChildVolume == 0 || secondChildVolume == 0) {

  //      cost = cost * 2;
   // }

    return cost;
}


std::set<int>  BoundingVolumeHierarchy::retrieveVerticesIndicesFromTrianglesIndices(std::vector<int> trianglesIndices) {

    const std::vector<glm::uvec3> triangles = m_pScene->meshes[0].triangles;

    std::set<int> verticesIndices;

    for (int index : trianglesIndices) {

        verticesIndices.insert(triangles[index][0]);
        verticesIndices.insert(triangles[index][1]);
        verticesIndices.insert(triangles[index][2]);
    }

    return verticesIndices;
}
