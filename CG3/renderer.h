#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <future>
#include "glm/vec3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vector_relational.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/intersect.hpp"

typedef glm::vec3 Color;
typedef glm::vec3 Vec3;
typedef glm::mat4 Mat4;

enum ObjectType {
	INVALID,
	SPHERE,
	TRIANGLE
};

struct ObjectId {
	ObjectType type;
	int index;
};

struct Material {
	Vec3 ambientFactor, diffuseFactor, specularFactor;
	float shininess;
	float reflectionFactor;
	bool refract;
	float refraction;
	float refractionFactor;
};

struct Sphere {
	Vec3 center;
	float radius;
	Material *material;
};

struct Triangle {
	Vec3 vertex[3];
	Vec3 norm;
	Material *material;
};

struct Camera {
	Vec3 position;
	Vec3 at;
	Vec3 up;
	float zNear, zFar;
	float fovy;
	float aspect;
};

enum LightType {
	LT_POINT,
	LT_DIRECTIONAL
};

struct Light {
	LightType type;
	Vec3 position;
	float intensity;
	Color color;
};

struct BoundingBox {
	Vec3 min, max;
};

struct OctreeNode {
	BoundingBox bounds;
	std::vector<int> objects;
	OctreeNode *subnodes[8];
	bool leaf;
};

struct Scene {
	std::vector<Sphere> spheres;
	std::vector<Triangle> triangles;
	std::vector<Light> lights;
	Camera camera;
	Color bgColor;
	OctreeNode octreeRoot;
};

struct RenderParams {
	bool enableOctree;
	int depthLimit;
	int width;
	int height;
	int threads;
};

void buildOctree(Scene &scene);
void render(const Scene &scene, unsigned int *pixels, const RenderParams &params);