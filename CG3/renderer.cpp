#include <iostream>
#include <future>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/intersect.hpp"
#include "renderer.h"

const int OCTREE_DEPTH = 7;
const int OCTREE_MAX_OBJ = 100;

struct Ray {
	Vec3 from;
	Vec3 dir;
	Vec3 inv_dir;

public:
	Ray(Vec3 from_, Vec3 dir_) : from(from_), dir(dir_), inv_dir({ 1.0f / dir_.x, 1.0f / dir_.y, 1.0f / dir_.z }) { }
};

static BoundingBox extend(const BoundingBox &bbox, float d) {
	BoundingBox e = {
			{ bbox.min.x - d, bbox.min.y - d, bbox.min.z - d },
			{ bbox.max.x + d, bbox.max.y + d, bbox.max.z + d }
	};
	return e;
}

static BoundingBox get_bbox(const Triangle &obj) {
	BoundingBox b = {
			{
				std::min({ obj.vertex[0].x, obj.vertex[1].x, obj.vertex[2].x }),
				std::min({ obj.vertex[0].y, obj.vertex[1].y, obj.vertex[2].y }),
				std::min({ obj.vertex[0].z, obj.vertex[1].z, obj.vertex[2].z })
			},
			{
				std::max({ obj.vertex[0].x, obj.vertex[1].x, obj.vertex[2].x }),
				std::max({ obj.vertex[0].y, obj.vertex[1].y, obj.vertex[2].y }),
				std::max({ obj.vertex[0].z, obj.vertex[1].z, obj.vertex[2].z })
			}
	};
	return b;
}

static bool overlaps(const BoundingBox &bbox, const BoundingBox &b) {
	return !((b.max.x < bbox.min.x)
		|| (b.max.y < bbox.min.y)
		|| (b.max.z < bbox.min.z)
		|| (b.min.x > bbox.max.x)
		|| (b.min.y > bbox.max.y)
		|| (b.min.z > bbox.max.z));
}

int stat_emptyNode = 0, stat_overMax = 0, stat_overDepth = 0;

static void splitOctreeNode(const Scene &scene, OctreeNode *node, int depth) {
	node->leaf = false;
	BoundingBox b = node->bounds;
	Vec3 center = (b.min + b.max) / 2.0f;
	float e = std::numeric_limits<float>().epsilon();
	BoundingBox bboxes[8] = {
			{ b.min, { center.x + e, center.y + e, center.z + e } },
			{ { center.x - e, b.min.y, b.min.z }, { b.max.x, center.y + e, center.z + e } },
			{ { center.x - e, b.min.y, center.z - e }, { b.max.x, center.y + e, b.max.z } },
			{ { b.min.x, b.min.y, center.z - e }, { center.x + e, center.y + e, b.max.z } },
			{ { b.min.x, center.y - e, b.min.z }, { center.x + e, b.max.y, center.z + e } },
			{ { center.x - e, center.y - e, b.min.z }, { b.max.x, b.max.y, center.z + e } },
			{ { center.x - e, center.y - e, center.z - e }, b.max },
			{ { b.min.x, center.y - e, center.z - e }, { center.x + e, b.max.y, b.max.z } }
	};
	for (int j = 0; j < 8; j++) {
		OctreeNode *subnode = new OctreeNode;
		subnode->leaf = true;
		subnode->bounds = bboxes[j];
		int n = 0;
		for (int i : node->objects) {
			if (overlaps(subnode->bounds, get_bbox(scene.triangles[i]))) {
				subnode->objects.push_back(i);
				n++;
			}
		}
		if (n == 0) {
			stat_emptyNode++;
			delete subnode;
			subnode = nullptr;
		}
		else if (depth == OCTREE_DEPTH) {
			stat_overDepth++;
		}
		else if (n < OCTREE_MAX_OBJ) {
			stat_overMax++;
		}
		else {
			splitOctreeNode(scene, subnode, depth + 1);
		}
		node->subnodes[j] = subnode;
	}
}

void buildOctree(Scene &scene) {
	float size = 10;
	scene.octreeRoot.bounds.min = { -size, -size, -size };
	scene.octreeRoot.bounds.max = { size, size, size };
	for (int i = 0; i < scene.triangles.size(); i++)
		scene.octreeRoot.objects.push_back(i);
	splitOctreeNode(scene, &scene.octreeRoot, 0);
	/*
	std::cout << "built octree: empty=" << stat_emptyNode <<
		" overDepth=" << stat_overDepth <<
		" overMax=" << stat_overMax << std::endl;
	*/
}

static void deleteNode(OctreeNode *node) {
	if (!node->leaf) {
		for (int i = 0; i < 8; i++) {
			if (node->subnodes[i] != nullptr) {
				deleteNode(node->subnodes[i]);
				delete node->subnodes[i];
				node->subnodes[i] = nullptr;
			}
		}
	}
}

void destroyOctree(Scene &scene) {
	scene.octreeRoot.objects.clear();
	deleteNode(&scene.octreeRoot);
}

static bool intersectBboxRay(const BoundingBox &bbox, const Ray &ray) {
	Vec3 a = (bbox.min - ray.from) * ray.inv_dir;
	Vec3 b = (bbox.max - ray.from) * ray.inv_dir;

	float tmin = std::max(std::max(std::min(a[0], b[0]), std::min(a[1], b[1])), std::min(a[2], b[2]));
	float tmax = std::min(std::min(std::max(a[0], b[0]), std::max(a[1], b[1])), std::max(a[2], b[2]));

	// if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behind us
	if (tmax < 0)
		return false;

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
		return false;

	return true;
}

static bool findNode(const Scene &scene, const OctreeNode *node, const Ray &ray, const int excludeId, int &nearestId, float &nearestDist) {
	bool found = false;
	if (!node->leaf) {
		for (OctreeNode *subnode : node->subnodes) {
			// Early pruning!
			if (subnode == nullptr || !intersectBboxRay(subnode->bounds, ray))
				continue;

			if (findNode(scene, subnode, ray, excludeId, nearestId, nearestDist))
				found = true;
		}
	}
	else {
		Vec3 baryPos;
		for (int i : node->objects) {
			if (i == excludeId) continue;
			const Triangle &obj = scene.triangles[i];
			if (glm::intersectRayTriangle(ray.from, ray.dir, obj.vertex[0], obj.vertex[1], obj.vertex[2], baryPos)) {
				if (baryPos.z < nearestDist) {
					found = true;
					nearestDist = baryPos.z;
					nearestId = i;
				}
			}
		}
	}
	return found;
}

static bool intersectRaySphere(const Ray &ray, const Vec3 &center, float radius, float &distance) {
	float len = glm::dot(ray.dir, center - ray.from);
	if (len < 0.f) // behind the ray
		return false;
	Vec3 d = center - (ray.from + ray.dir * len);
	float dst2 = glm::dot(d, d);
	float r2 = radius * radius;
	if (dst2 > r2) return false;
	distance = len - sqrt(r2 - dst2);
	return true;
}

static bool _findNearestObject(const Scene &scene, const RenderParams &params, const Ray &ray, const ObjectId excludeObjectID, bool excludeTransparentMat, ObjectId &nearestObjectID, float &nearestDist, bool &isInside) {
	bool found = false;
	for (int i = 0; i < scene.spheres.size(); i++) {
		if (excludeObjectID.type == SPHERE && i == excludeObjectID.index)
			continue;
		const Sphere &obj = scene.spheres[i];
		if (excludeTransparentMat && obj.material->refract)
			continue;
		float distance;
		if (intersectRaySphere(ray, obj.center, obj.radius, distance)) {
			if (distance < nearestDist) {
				found = true;
				nearestDist = distance;
				nearestObjectID.type = SPHERE;
				nearestObjectID.index = i;
				isInside = glm::distance(ray.from, obj.center) < obj.radius;
			}
		}
	}
	if (params.enableOctree) {
		if (findNode(scene, &scene.octreeRoot, ray, excludeObjectID.type == TRIANGLE ? excludeObjectID.index : -1, nearestObjectID.index, nearestDist)) {
			found = true;
			nearestObjectID.type = TRIANGLE;
			isInside = false;
		}
	}
	else {
		for (int i = 0; i < scene.triangles.size(); i++) {
			if (excludeObjectID.type == TRIANGLE && i == excludeObjectID.index)
				continue;
			const Triangle &obj = scene.triangles[i];
			if (excludeTransparentMat && obj.material->refract)
				continue;
			Vec3 baryPos;
			if (glm::intersectRayTriangle(ray.from, ray.dir, obj.vertex[0], obj.vertex[1], obj.vertex[2], baryPos)) {
				// See https://github.com/g-truc/glm/issues/6
				float distance = baryPos.z;
				if (distance < nearestDist) {
					found = true;
					nearestDist = distance;
					nearestObjectID.type = TRIANGLE;
					nearestObjectID.index = i;
					isInside = false;
				}
			}
		}
	}
	return found;
}

static bool findNearestObject(const Scene &scene, const RenderParams &params, const Ray &ray, const ObjectId excludeObjectID, bool excludeTransparentMat, ObjectId &nearestObjectID, Vec3 &nearestPos, Vec3 &nearestNorm, Material **nearestMat, bool &isInside) {
	float nearestDist = std::numeric_limits<float>::max();
	if (_findNearestObject(scene, params, ray, excludeObjectID, excludeTransparentMat, nearestObjectID, nearestDist, isInside)) {
		if (nearestObjectID.type == SPHERE) {
			const Sphere &obj = scene.spheres[nearestObjectID.index];
			nearestPos = ray.from + ray.dir * nearestDist;
			nearestNorm = (nearestPos - obj.center) / obj.radius;
			*nearestMat = obj.material;
		}
		else if (nearestObjectID.type == TRIANGLE) {
			const Triangle &obj = scene.triangles[nearestObjectID.index];
			nearestPos = ray.from + ray.dir * nearestDist;
			nearestNorm = obj.norm;
			*nearestMat = obj.material;
			isInside = false;
		}
		return true;
	}
	return false;
}

bool isShaded(const Scene &scene, const RenderParams &params, const Ray &ray, const ObjectId &excludeObjectID) {
	ObjectId a;
	float d;
	bool isInside;
	return _findNearestObject(scene, params, ray, excludeObjectID, true, a, d, isInside);
}

Color _renderPixel(const Scene &scene, const RenderParams &params, const Ray &ray, ObjectId prevObjectID, int depth, float rIndex) {
	ObjectId objectID;
	Vec3 pos, norm;
	Material *m;
	bool isInside = false;
	if (!findNearestObject(scene, params, ray, prevObjectID, false, objectID, pos, norm, &m, isInside)) {
		return scene.bgColor;
	}

	Color texture = { 1.0, 1.0, 1.0 };
	if (m->texFunc && objectID.type == TRIANGLE) {
		const Triangle &obj = scene.triangles[objectID.index];
		Vec3 f1 = obj.vertex[0] - pos;
		Vec3 f2 = obj.vertex[1] - pos;
		Vec3 f3 = obj.vertex[2] - pos;
		float a = glm::length(glm::cross(obj.vertex[0] - obj.vertex[1], obj.vertex[1] - obj.vertex[2]));
		float a1 = glm::length(glm::cross(f2, f3)) / a;
		float a2 = glm::length(glm::cross(f3, f1)) / a;
		float a3 = glm::length(glm::cross(f1, f2)) / a;
		texture = m->texFunc(obj.texCoord[0] * a1 + obj.texCoord[1] * a2 + obj.texCoord[2] * a3);
	}
	Color c = scene.bgColor * m->ambientFactor * texture;
	Vec3 reflectionDir = glm::normalize(glm::reflect(ray.dir, norm));
	for (const Light &light : scene.lights) {
		Vec3 lightDir;
		if (light.type == LT_POINT) {
			lightDir = glm::normalize(light.position - pos);
		}
		else if (light.type == LT_DIRECTIONAL) {
			lightDir = -light.position;
		}
		else if (light.type == LT_SPOT) {
			lightDir = glm::normalize(light.position - pos);
			float p = glm::dot(-lightDir, light.spotDir);
			if (p < light.spotCutoff)
				continue;
		}

		float s = glm::dot(norm, lightDir);
		if (s > 0.0f && !isShaded(scene, params, { pos, lightDir }, objectID)) {
			Color diffuse(s * light.intensity * texture);
			c += diffuse * light.color * m->diffuseFactor;
		}

		float t = glm::dot(lightDir, reflectionDir);
		if (t > 0.0f && !isShaded(scene, params, { pos, reflectionDir }, objectID)) {
			Color specular = Color(powf(t, m->shininess) * light.intensity);
			c += specular * light.color * m->specularFactor;
		}
	}

	if (depth < params.depthLimit) {
		c += _renderPixel(scene, params, { pos, reflectionDir }, objectID, depth + 1, rIndex) * m->reflectionFactor;
		if (m->refract) {
			float n = rIndex / m->refraction;
			Vec3 N = norm;
			if (isInside)
				N *= -1;
			float cosI = glm::dot(N, ray.dir);
			float cosT2 = 1.0f - n * n * (1.0f - cosI * cosI);
			Color r;
			if (cosT2 > 0.0f) {
				Vec3 refractionDir = n * ray.dir + (n * cosI - sqrtf(cosT2)) * N;
				// For refraction we don't exclude current object
				r = _renderPixel(scene, params, { pos + refractionDir * 1e-5f, refractionDir }, {}, depth + 1, m->refraction) * m->refractionFactor;
			}
			c = c * (1 - m->refractionFactor) + r;
		}
	}
	return glm::clamp(c, 0.0f, 1.0f);
}

static void _render(const Scene &scene, unsigned int *pixels, const RenderParams &params, int threadid, const Mat4 &proj, const glm::vec4 &viewport) {
	Mat4 model;
	for (int y = threadid; y < params.height; y += params.threads) {
		for (int x = 0; x < params.width; x++) {
			Vec3 win = { x, y, 0 };
			Vec3 p = glm::unProject(win, model, proj, viewport);
			Color c = _renderPixel(scene, params, { scene.camera.position, glm::normalize(p - scene.camera.position) }, {}, 0, 1.0f);
			pixels[y * params.width + x] = ((unsigned int)(255 * c.r) & 0xFF) << 16 | ((unsigned int)(255 * c.g) & 0xFF) << 8 | ((unsigned int)(255 * c.b) & 0xFF);
		}
	}
}

void render(const Scene &scene, unsigned int *pixels, const RenderParams &params) {
	Mat4 proj = glm::perspective(scene.camera.fovy * 3.14159265358979323846f / 180.0f, scene.camera.aspect, scene.camera.zNear, scene.camera.zFar) *
		glm::lookAt(scene.camera.position, scene.camera.at, scene.camera.up);
	glm::vec4 viewport(0, 0, params.width, params.height);
	std::vector<std::future<void>> tasks;
	for (int i = 0; i < params.threads; i++) {
		tasks.push_back(std::async(std::launch::async, _render, scene, pixels, params, i, proj, viewport));
	}
	for (int i = 0; i < tasks.size(); i++) {
		tasks[i].get();
	}
}
