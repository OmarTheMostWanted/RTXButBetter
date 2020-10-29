#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <iostream>
#include <numeric>
#include <set>
#include <climits>
#include <glm/geometric.hpp>

// Initializes the BVH class.
// Creates all the parent nodes - one parent node for each mesh.
// Calls the recursive function splitNode for each of the parent nodes which then splits the node according to the number of levels.
BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{

    int treeHeight = 0;
    for (int i = 0; i < pScene->meshes.size(); i++) {

        createParentNode(i);

        if (SPLITS_PER_NODE == 0) {

            treeHeight = fmax(treeHeight, splitNodeCentroid(parentNodes[i], 0));
        }
        else {

            treeHeight = fmax(treeHeight, splitNodeConst(parentNodes[i], 0));
        }
        
    }

    this->treeLevels = treeHeight;
}

// Starts drawing of the boxes contained in the BVH.
// Only nodes existing at the particural level will be displayed.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // First nodes in the nodes vector are the parent nodes.
    for (int i = 0; i < parentNodes.size(); i++) {

        drawNode(parentNodes[i], level);
    }
}

// Draws a box contained in a node if the level corresponds to the level provided in the debugDraw method(green).
// Otherwise delves deeper into the BVH tree to find the node corresponding to the given level.
// Nodes which are leaf nodes and reside on lower levels of the tree are colored blue.
void BoundingVolumeHierarchy::drawNode(int nodeIndex, int remainingLevels) {

    
    Node& node = this->nodes[nodeIndex];
    if (remainingLevels == 0) {

        drawAABB(node.box, DrawMode::Wireframe, glm::vec3(0.0f, 1.0f, 0.0f), 1.0);
        return;
    }
    else if (node.type == 1) {

        drawAABB(node.box, DrawMode::Wireframe, glm::vec3(0.0f, 0.0f, 1.0f), 1.0);
        return;
    }

    if (node.type == 0) {

        for (int i : node.indices) {

            drawNode(i, remainingLevels - 1);
        }
    }
    

}

// Returns number of levels. The more levels the more nodes in the BVH.
int BoundingVolumeHierarchy::numLevels() const
{
    return treeLevels;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
//bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
//{
//    bool hit = false;
//    // Intersect with all triangles of all meshes.
//    for (const auto& mesh : m_pScene->meshes) {
//        for (const auto& tri : mesh.triangles) {
//            const auto v0 = mesh.vertices[tri[0]];
//            const auto v1 = mesh.vertices[tri[1]];
//            const auto v2 = mesh.vertices[tri[2]];
//            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
//                hitInfo.material = mesh.material;
//                hit = true;
//            }
//        }
//    }
//    // Intersect with spheres.
//    for (const auto& sphere : m_pScene->spheres)
//        hit |= intersectRayWithShape(sphere, ray, hitInfo);
//    return hit;
//}

// Checks intersection with all nodes and returns true if any intersection occurs.
// Calls intersectWithNodes function recursively.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, int level, int max) const
{
    if ( max < level) return false;
    bool hit = false;

    // intersection with spheres
    for (const auto& sphere : m_pScene->spheres) {

        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    }

    for (int i = 0; i < parentNodes.size(); i++) {

    hit = hit | intersectWithNodes(parentNodes[i], ray, hitInfo);
    }

    // intersection with boxes
    for (const auto& box : m_pScene->boxes) {

        hit |= intersectRayWithShape(box, ray);
    }
       

    return hit;
}


// Recursively checks if a ray intersects with nodes.
// Return true if it does. Calls intersectWithTriangles if the nodes ia a leaf node.
bool BoundingVolumeHierarchy::intersectWithNodes(int nodeIndex, Ray& ray, HitInfo& hitInfo) const{

    Node currentNode = nodes[nodeIndex];

    if (!intersectRayWithNodeBox(currentNode.box, ray)) {

        return false;
    }

    if (currentNode.type == true) {

        return intersectWithTriangles(nodeIndex, ray, hitInfo);
    }

    bool hit = false;

    hit = hit | intersectWithNodes(currentNode.indices[0], ray, hitInfo);
    hit = hit | intersectWithNodes(currentNode.indices[1], ray, hitInfo);
   
    return hit;
}

// Checks intersection of a ray with all the triangles contained in a leaf node.
// Returns true if the intersection occurs.
bool BoundingVolumeHierarchy::intersectWithTriangles(int nodeIndex, Ray& ray, HitInfo& hitInfo) const {

    Node leaf = nodes[nodeIndex];

    if (leaf.type == false) {

        return false;
    }

    std::vector<Vertex>& vertices = this->m_pScene->meshes[leaf.meshIndex].vertices;
    std::vector<Triangle>& triangles = this->m_pScene->meshes[leaf.meshIndex].triangles;
    bool result = 0;

    for (int i : leaf.indices) {

        result = result | intersectRayWithTriangle(vertices[triangles[i][0]], vertices[triangles[i][1]], vertices[triangles[i][2]],
            ray, hitInfo, m_pScene->meshes[leaf.meshIndex].material);
    }

    return result;
}

// Creates new AxisAlignedBox containing all of the vertices with given indices and returns it.
AxisAlignedBox BoundingVolumeHierarchy::createBoxFromTriangles(std::vector<int> triangles, int meshIndex)
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

    const std::vector<Vertex>& vertices = m_pScene->meshes[meshIndex].vertices;

    std::set<int> verticesIndices = retrieveVerticesIndicesFromTrianglesIndices(triangles, meshIndex);

    

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

// Create a parent node and add it to the main vector of vertices.
Node& BoundingVolumeHierarchy::createParentNode(int meshNumber) {

    float minX = INT_MAX;
    float minY = INT_MAX;
    float minZ = INT_MAX;

    float maxX = INT_MIN;
    float maxY = INT_MIN;
    float maxZ = INT_MIN;

    Mesh& mesh = this->m_pScene->meshes[meshNumber];

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

    std::vector<int> allIndices(m_pScene->meshes[meshNumber].triangles.size());
    std::iota(begin(allIndices), end(allIndices), 0);

    Node parentNode{ allIndices, 1, newBox, FLT_MAX, meshNumber };
    nodes.push_back(parentNode);
    parentNodes.push_back(nodes.size() - 1);

    return parentNode;
}

// Creates a new node out of given vertices with given indices.
Node BoundingVolumeHierarchy::createNodeFromTriangles(std::vector<int> triangles, int meshIndex)
{
    Node newNode;
    newNode.box = createBoxFromTriangles(triangles, meshIndex);
    newNode.indices = triangles; // this will be replaces with the indices of children nodes if the node turns out to be an interior node
    newNode.type = 1; // a temporal status
    newNode.splitCost = FLT_MAX;

    return newNode;
}

// Creates a new node out of given vertices given as indices and an already made box.
Node BoundingVolumeHierarchy::createNodeFromTriangles(std::vector<int> triangles, int meshIndex, AxisAlignedBox& box)
{

    Node newNode;
    newNode.box = box;
    newNode.indices = triangles; // this will be replaces with the indices of children nodes if the node turns out to be an interior node
    newNode.type = 1; // a temporal status
    newNode.splitCost = FLT_MAX;

    return newNode;
}

// Calculates the best splits of a node along all the 3 axes.
// Each axis is being checked for SPLITS_PER_NODE - 1 different divisors.
int BoundingVolumeHierarchy::splitNodeConst(int nodeIndex, int treeDepth) {

    if (nodes[nodeIndex].indices.size() <= MAX_TRIANGLES_IN_LEAF) {

        return treeDepth;
    }

    Node& node = nodes[nodeIndex];
    std::vector<Vertex>& vertices = m_pScene->meshes[node.meshIndex].vertices;
    std::vector<Triangle>& triangles = m_pScene->meshes[node.meshIndex].triangles;

    float lowerX = node.box.lower[0];
    float higherX = node.box.upper[0];

    float lowerY = node.box.lower[1];
    float higherY = node.box.upper[1];

    float lowerZ = node.box.lower[2];
    float higherZ = node.box.upper[2];

    float rangeX = higherX - lowerX;
    float rangeY = higherY - lowerY;
    float rangeZ = higherZ - lowerZ;

    float intervalX = rangeX / (float) (SPLITS_PER_NODE + 1);
    float intervalY = rangeY / (float)(SPLITS_PER_NODE + 1);
    float intervalZ = rangeZ / (float)(SPLITS_PER_NODE + 1);

    // Creating vectors to hold resulting triangles after each split
    std::vector<std::vector<std::vector<int>>> resultingTrianglesX(SPLITS_PER_NODE);
    std::vector<std::vector<std::vector<int>>> resultingTrianglesY(SPLITS_PER_NODE);
    std::vector<std::vector<std::vector<int>>> resultingTrianglesZ(SPLITS_PER_NODE);

     for (int i = 0; i < SPLITS_PER_NODE; i++) {

        float pointX = lowerX + (i + 1) * intervalX;
        float pointY =  lowerY + (i + 1) * intervalY;
        float pointZ = lowerZ + (i + 1) * intervalZ;

        resultingTrianglesX[i] = divideByPlane(nodeIndex, 'X', pointX);
        resultingTrianglesY[i] = divideByPlane(nodeIndex, 'Y', pointY);
        resultingTrianglesZ[i] = divideByPlane(nodeIndex, 'Z', pointZ);


    }

    for (int i = 0; i < SPLITS_PER_NODE; i++) {

        compareCostsAndUpdate(nodeIndex, resultingTrianglesX[i]);
        compareCostsAndUpdate(nodeIndex, resultingTrianglesY[i]);
        compareCostsAndUpdate(nodeIndex, resultingTrianglesZ[i]);
    }

    Node& parentNode = nodes[nodeIndex];
    Node& firstChild = nodes[nodes[nodeIndex].indices[0]];
    Node& secondChild = nodes[nodes[nodeIndex].indices[1]];

    if (nodes[nodeIndex].box.lower == firstChild.box.lower && nodes[nodeIndex].box.upper == firstChild.box.upper) {

        return treeDepth;
    }

    if (nodes[nodeIndex].box.lower == secondChild.box.lower && nodes[nodeIndex].box.upper == secondChild.box.upper) {

        return treeDepth;
    }

    int newHeight1 = splitNodeConst(nodes[nodeIndex].indices[0], treeDepth + 1);
    int newHeight2 = splitNodeConst(nodes[nodeIndex].indices[1], treeDepth + 1);

    return fmax(newHeight1, newHeight2);
}

int BoundingVolumeHierarchy::splitNodeCentroid(int nodeIndex, int treeDepth) {

    if (nodes[nodeIndex].indices.size() <= MAX_TRIANGLES_IN_LEAF) {

        return treeDepth;
    }

    Node& node = nodes[nodeIndex];
    std::vector<Vertex>& vertices = m_pScene->meshes[node.meshIndex].vertices;
    std::vector<Triangle>& triangles = m_pScene->meshes[node.meshIndex].triangles;

   

    glm::vec3 sumOfCentroids(0);
    for (int index : node.indices) {

        sumOfCentroids += (vertices[triangles[index][0]].p + vertices[triangles[index][1]].p + vertices[triangles[index][2]].p)/3.0f;
    }
    glm::vec3 averageOfCentroids = sumOfCentroids / (float) node.indices.size();

    

    std::vector<std::vector<int>> resultingTrianglesX = divideByPlaneX(nodeIndex, averageOfCentroids[0]);
    std::vector<std::vector<int>> resultingTrianglesY = divideByPlaneY(nodeIndex, averageOfCentroids[1]);
    std::vector<std::vector<int>> resultingTrianglesZ = divideByPlaneZ(nodeIndex, averageOfCentroids[2]);
    compareCostsAndUpdate(nodeIndex, resultingTrianglesX);
    compareCostsAndUpdate(nodeIndex, resultingTrianglesY);
    compareCostsAndUpdate(nodeIndex, resultingTrianglesZ);

    Node& parentNode = nodes[nodeIndex];
    Node& firstChild = nodes[nodes[nodeIndex].indices[0]];
    Node& secondChild = nodes[nodes[nodeIndex].indices[1]];

    if (nodes[nodeIndex].box.lower == firstChild.box.lower && nodes[nodeIndex].box.upper == firstChild.box.upper) {

        return treeDepth;
    }

    if (nodes[nodeIndex].box.lower == secondChild.box.lower && nodes[nodeIndex].box.upper == secondChild.box.upper) {

        return treeDepth;
    }

    
    int newHeight1 = splitNodeCentroid(nodes[nodeIndex].indices[0], treeDepth + 1);
    int newHeight2 = splitNodeCentroid(nodes[nodeIndex].indices[1], treeDepth + 1);
    
    return fmax(newHeight1, newHeight2);
}

// Compares cost of performing a new split to the already existing one and updates it if the new cost turns out to be lower
// Takes the parent node and a vector containing two groups of vertices' indices resulting from the division
void BoundingVolumeHierarchy::compareCostsAndUpdate(int nodeIndex, std::vector<std::vector<int>> dividedTriangles) {

    Node& parentNode = nodes[nodeIndex];
    AxisAlignedBox firstBox = createBoxFromTriangles(dividedTriangles[0], parentNode.meshIndex);
    AxisAlignedBox secondBox = createBoxFromTriangles(dividedTriangles[1], parentNode.meshIndex);

    float newCost = calculateSplitCost(firstBox, secondBox);

    if (newCost < parentNode.splitCost) {

        parentNode.splitCost = newCost;
        Node firstChild = createNodeFromTriangles(dividedTriangles[0], parentNode.meshIndex, firstBox);
        Node secondChild = createNodeFromTriangles(dividedTriangles[1], parentNode.meshIndex, secondBox);
       
        replaceChildren(nodeIndex, firstChild, secondChild);
        
    }
}

// Replaces children of a node. Updates the content of the main nodes vector with the newly added nodes.
void BoundingVolumeHierarchy::replaceChildren(int parentNodeIndex, Node& firstChild, Node& secondChild) {

    
    if (nodes[parentNodeIndex].type == 0) {

        this->nodes.pop_back();
        this->nodes.pop_back();
    }

    firstChild.meshIndex = nodes[parentNodeIndex].meshIndex;
    secondChild.meshIndex = nodes[parentNodeIndex].meshIndex;
    this->nodes.push_back(firstChild);
    this->nodes.push_back(secondChild);

    // pushing the indices of newly created node to be our children nodes

    this->nodes[parentNodeIndex].indices.clear();
    this->nodes[parentNodeIndex].indices.push_back(this->nodes.size() - 2);
    this->nodes[parentNodeIndex].indices.push_back(this->nodes.size() - 1);
    this->nodes[parentNodeIndex].type = 0;

}

// Divides the node along the given plane.
// Plane is characterised by a normal and any point.
// Returns vector of two vectors containing indicies of two groups of verticies resulting from the split.
std::vector<std::vector<int>> BoundingVolumeHierarchy::divideByPlane(int nodeIndex, char axis, float point) {

    std::vector<int> firstGroupTriangles;
    std::vector<int> secondGroupTriangles;

    int meshIndex = nodes[nodeIndex].meshIndex;
    Mesh mesh = m_pScene->meshes[meshIndex];
    std::vector<glm::uvec3> triangles = m_pScene->meshes[meshIndex].triangles;
    std::vector<Vertex> vertices = m_pScene->meshes[meshIndex].vertices;

    for (int triangleIndex: nodes[nodeIndex].indices){

        glm::uvec3 triangle = triangles[triangleIndex];

            /*bool check1 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector1) >= 0;
            bool check2 = glm::dot(normal, controlVector1) >= 0 && glm::dot(normal, controlVector2) >= 0;
            bool check3 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector2) >= 0;*/
            
        if (axis == 'X' || axis == 'x') {

            if (vertices[triangle[0]].p[0] >= point || vertices[triangle[1]].p[0] >= point || vertices[triangle[2]].p[0] >= point) {


                firstGroupTriangles.push_back(triangleIndex);
            }

            else {

                secondGroupTriangles.push_back(triangleIndex);
            }
        }
        else if (axis == 'Y' || axis == 'y') 
        {
            if (vertices[triangle[0]].p[1] >= point || vertices[triangle[1]].p[1] >= point || vertices[triangle[2]].p[1] >= point) {


                firstGroupTriangles.push_back(triangleIndex);
            }

            else {

                secondGroupTriangles.push_back(triangleIndex);
            }
        }
        else  {

            if (vertices[triangle[0]].p[2] >= point || vertices[triangle[1]].p[2] >= point || vertices[triangle[2]].p[2] >= point) {


                firstGroupTriangles.push_back(triangleIndex);
            }

            else {

                secondGroupTriangles.push_back(triangleIndex);
            }
        }
        
    }

    std::vector<std::vector<int>> result;
    result.push_back(firstGroupTriangles);
    result.push_back(secondGroupTriangles);

    return result;

}

std::vector<std::vector<int>> BoundingVolumeHierarchy::divideByPlaneX(int nodeIndex, float pointX) {

    std::vector<int> firstGroupTriangles;
    std::vector<int> secondGroupTriangles;

    int meshIndex = nodes[nodeIndex].meshIndex;
    Mesh& mesh = m_pScene->meshes[meshIndex];
    std::vector<glm::uvec3>& triangles = m_pScene->meshes[meshIndex].triangles;
    std::vector<Vertex>& vertices = m_pScene->meshes[meshIndex].vertices;

    for (int triangleIndex : nodes[nodeIndex].indices) {

        float triangleXCoord1 = vertices[triangles[triangleIndex][0]].p[0];
        float triangleXCoord2 = vertices[triangles[triangleIndex][1]].p[0];
        float triangleXCoord3 = vertices[triangles[triangleIndex][2]].p[0];

        /*bool check1 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector1) >= 0;
        bool check2 = glm::dot(normal, controlVector1) >= 0 && glm::dot(normal, controlVector2) >= 0;
        bool check3 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector2) >= 0;*/


        if (triangleXCoord1 >= pointX || triangleXCoord2 >= pointX || triangleXCoord3 >= pointX) {

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

std::vector<std::vector<int>> BoundingVolumeHierarchy::divideByPlaneY(int nodeIndex, float pointY) {

    std::vector<int> firstGroupTriangles;
    std::vector<int> secondGroupTriangles;

    int meshIndex = nodes[nodeIndex].meshIndex;
    Mesh& mesh = m_pScene->meshes[meshIndex];
    std::vector<glm::uvec3>& triangles = m_pScene->meshes[meshIndex].triangles;
    std::vector<Vertex>& vertices = m_pScene->meshes[meshIndex].vertices;

    for (int triangleIndex : nodes[nodeIndex].indices) {

        float triangleYCoord1 = vertices[triangles[triangleIndex][0]].p[1];
        float triangleYCoord2 = vertices[triangles[triangleIndex][1]].p[1];
        float triangleYCoord3 = vertices[triangles[triangleIndex][2]].p[1];

        /*bool check1 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector1) >= 0;
        bool check2 = glm::dot(normal, controlVector1) >= 0 && glm::dot(normal, controlVector2) >= 0;
        bool check3 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector2) >= 0;*/


        if (triangleYCoord1 >= pointY || triangleYCoord2 >= pointY || triangleYCoord3 >= pointY) {

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

std::vector<std::vector<int>> BoundingVolumeHierarchy::divideByPlaneZ(int nodeIndex, float pointZ) {

    std::vector<int> firstGroupTriangles;
    std::vector<int> secondGroupTriangles;

    int meshIndex = nodes[nodeIndex].meshIndex;
    Mesh& mesh = m_pScene->meshes[meshIndex];
    std::vector<glm::uvec3>& triangles = m_pScene->meshes[meshIndex].triangles;
    std::vector<Vertex>& vertices = m_pScene->meshes[meshIndex].vertices;

    for (int triangleIndex : nodes[nodeIndex].indices) {

        float triangleZCoord1 = vertices[triangles[triangleIndex][0]].p[2];
        float triangleZCoord2 = vertices[triangles[triangleIndex][1]].p[2];
        float triangleZCoord3 = vertices[triangles[triangleIndex][2]].p[2];

        /*bool check1 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector1) >= 0;
        bool check2 = glm::dot(normal, controlVector1) >= 0 && glm::dot(normal, controlVector2) >= 0;
        bool check3 = glm::dot(normal, controlVector0) >= 0 && glm::dot(normal, controlVector2) >= 0;*/


        if (triangleZCoord1 >= pointZ || triangleZCoord2 >= pointZ || triangleZCoord3 >= pointZ) {

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

// Calculates the volume of a given box.
float BoundingVolumeHierarchy::calculateBoxVolume(AxisAlignedBox& box) {

    return (box.upper[0] - box.lower[0]) * (box.upper[1] - box.lower[1]) * (box.upper[2] - box.lower[2]);
}

// Calculates the cost of a split
// the smaller the cost, the better the split is
float BoundingVolumeHierarchy::calculateSplitCost(AxisAlignedBox firstChild, AxisAlignedBox secondChild ) {

    float firstChildVolume = calculateBoxVolume(firstChild);
    float secondChildVolume = calculateBoxVolume(secondChild);

    float cost = firstChildVolume + secondChildVolume;

    if (firstChildVolume == 0 || secondChildVolume == 0) {

        cost = cost * 2;
    }

    return cost;
}


// Collects indices of all vertices that triangles with given indices contain and returns them as a set.
std::set<int>  BoundingVolumeHierarchy::retrieveVerticesIndicesFromTrianglesIndices(std::vector<int> trianglesIndices, int meshIndex) {

    const std::vector<glm::uvec3>& triangles = m_pScene->meshes[meshIndex].triangles;

    std::set<int> verticesIndices;

    for (int index : trianglesIndices) {

        verticesIndices.insert(triangles[index][0]);
        verticesIndices.insert(triangles[index][1]);
        verticesIndices.insert(triangles[index][2]);
    }

    return verticesIndices;
}

//// Calculates the optimal height of the tree
//// Uses the logBase as a base of the logarithm in the calculations
//int BoundingVolumeHierarchy::calculateTreeHeight(float logBase) {
//
//    int level = 0;
//
//    for (Mesh mesh : m_pScene->meshes) {
//
//        int temp = ceil(log(mesh.triangles.size()) / log(logBase));
//        if (temp > level) {
//
//            level = temp;
//        }
//    }
//
//    return level;
//}