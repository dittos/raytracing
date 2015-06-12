#include <iostream>
#include <future>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/intersect.hpp"
#include "renderer.h"

const int OCTREE_DEPTH = 7;
const int OCTREE_MAX_OBJ = 100;

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
	return extend(b, 1e-2f);
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
	BoundingBox bboxes[8] = {
			{ b.min, center },
			{ { center.x, b.min.y, b.min.z }, { b.max.x, center.y, center.z } },
			{ { center.x, b.min.y, center.z }, { b.max.x, center.y, b.max.z } },
			{ { b.min.x, b.min.y, center.z }, { center.x, center.y, b.max.z } },
			{ { b.min.x, center.y, b.min.z }, { center.x, b.max.y, center.z } },
			{ { center.x, center.y, b.min.z }, { b.max.x, b.max.y, center.z } },
			{ center, b.max },
			{ { b.min.x, center.y, center.z }, { center.x, b.max.y, b.max.z } }
	};
	for (int j = 0; j < 8; j++) {
		OctreeNode *subnode = new OctreeNode;
		subnode->leaf = true;
		subnode->bounds = extend(bboxes[j], 1e-2f);
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
		else if (depth >= OCTREE_DEPTH) {
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
	std::cout << "built octree: empty=" << stat_emptyNode <<
		" overDepth=" << stat_overDepth <<
		" overMax=" << stat_overMax << std::endl;
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

static bool intersectRayPlane(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const Vec3 &planeNormal, float planeD, Vec3 &p) {
	float d = glm::dot(normalizedRayDir, planeNormal);
	if (d != 0) {
		float t = -(glm::dot(rayFrom, planeNormal) - planeD) / d;
		if (t < 0) return false;
		p = rayFrom + normalizedRayDir * t;
		return true;
	}
	else if (glm::dot(rayFrom, planeNormal) == planeD) {
		p = rayFrom;
		return true;
	}
	return false;
}

static bool intersectBboxRay(const BoundingBox &bbox, const Vec3 &rayFrom, const Vec3 &normalizedRayDir) {
	if (bbox.min.x < rayFrom.x && rayFrom.x < bbox.max.x &&
		bbox.min.y < rayFrom.y && rayFrom.y < bbox.max.y &&
		bbox.min.z < rayFrom.z && rayFrom.z < bbox.max.z)
		// bbox contains rayFrom
		return true;

	Vec3 p;

	if (intersectRayPlane(rayFrom, normalizedRayDir, Vec3({ 0, 0, 1 }), bbox.min.z, p)
		&& (p.x > bbox.min.x) && (p.x < bbox.max.x)
		&& (p.y > bbox.min.y) && (p.y < bbox.max.y)) {
		return true;
	}

	if (intersectRayPlane(rayFrom, normalizedRayDir, Vec3({ 0, 0, 1 }), bbox.max.z, p)
		&& (p.x > bbox.min.x) && (p.x < bbox.max.x)
		&& (p.y > bbox.min.y) && (p.y < bbox.max.y)) {
		return true;
	}

	if (intersectRayPlane(rayFrom, normalizedRayDir, Vec3({ 0, 1, 0 }), bbox.min.y, p)
		&& (p.x > bbox.min.x) && (p.x < bbox.max.x)
		&& (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
		return true;
	}

	if (intersectRayPlane(rayFrom, normalizedRayDir, Vec3({ 0, 1, 0 }), bbox.max.y, p)
		&& (p.x > bbox.min.x) && (p.x < bbox.max.x)
		&& (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
		return true;
	}

	if (intersectRayPlane(rayFrom, normalizedRayDir, Vec3({ 1, 0, 0 }), bbox.min.z, p)
		&& (p.y > bbox.min.y) && (p.y < bbox.max.y)
		&& (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
		return true;
	}

	if (intersectRayPlane(rayFrom, normalizedRayDir, Vec3({ 1, 0, 0 }), bbox.max.z, p)
		&& (p.y > bbox.min.y) && (p.y < bbox.max.y)
		&& (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
		return true;
	}

	return false;
}

static bool findNode(const Scene &scene, const OctreeNode *node, const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const int excludeId, int &nearestId, float &nearestDist) {
	bool found = false;
	if (!node->leaf) {
		for (OctreeNode *subnode : node->subnodes) {
			// Early pruning!
			if (subnode == nullptr || !intersectBboxRay(subnode->bounds, rayFrom, normalizedRayDir))
				continue;

			if (findNode(scene, subnode, rayFrom, normalizedRayDir, excludeId, nearestId, nearestDist))
				found = true;
		}
	}
	else {
		Vec3 baryPos;
		for (int i : node->objects) {
			if (i == excludeId) continue;
			const Triangle &obj = scene.triangles[i];
			if (glm::intersectRayTriangle(rayFrom, normalizedRayDir, obj.vertex[0], obj.vertex[1], obj.vertex[2], baryPos)) {
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

static bool intersectRaySphere(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const Vec3 &center, float radius, float &distance) {
	float len = glm::dot(normalizedRayDir, center - rayFrom);
	if (len < 0.f) // behind the ray
		return false;
	Vec3 d = center - (rayFrom + normalizedRayDir * len);
	float dst2 = glm::dot(d, d);
	float r2 = radius * radius;
	if (dst2 > r2) return false;
	distance = len - sqrt(r2 - dst2);
	return true;
}

static bool findNearestObject(const Scene &scene, const RenderParams &params, const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const ObjectId excludeObjectID, bool excludeTransparentMat, ObjectId &nearestObjectID, Vec3 &nearestPos, Vec3 &nearestNorm, Material **nearestMat, bool &isInside) {
	bool found = false;
	float nearestDist = std::numeric_limits<float>::max();
	for (int i = 0; i < scene.spheres.size(); i++) {
		if (excludeObjectID.type == SPHERE && i == excludeObjectID.index)
			continue;
		const Sphere &obj = scene.spheres[i];
		if (excludeTransparentMat && obj.material->refract)
			continue;
		float distance;
		if (intersectRaySphere(rayFrom, normalizedRayDir, obj.center, obj.radius, distance)) {
			if (distance < nearestDist) {
				found = true;
				nearestDist = distance;
				nearestObjectID.type = SPHERE;
				nearestObjectID.index = i;
				nearestPos = rayFrom + normalizedRayDir * distance;
				nearestNorm = (nearestPos - obj.center) / obj.radius;
				*nearestMat = obj.material;
				isInside = glm::distance(rayFrom, obj.center) < obj.radius;
			}
		}
	}
	if (params.enableOctree) {
		if (findNode(scene, &scene.octreeRoot, rayFrom, normalizedRayDir, excludeObjectID.type == TRIANGLE ? excludeObjectID.index : -1, nearestObjectID.index, nearestDist)) {
			found = true;
			nearestObjectID.type = TRIANGLE;
			nearestPos = rayFrom + normalizedRayDir * nearestDist;
			nearestNorm = scene.triangles[nearestObjectID.index].norm;
			*nearestMat = scene.triangles[nearestObjectID.index].material;
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
			if (glm::intersectRayTriangle(rayFrom, normalizedRayDir, obj.vertex[0], obj.vertex[1], obj.vertex[2], baryPos)) {
				// See https://github.com/g-truc/glm/issues/6
				float distance = baryPos.z;
				if (distance < nearestDist) {
					found = true;
					nearestDist = distance;
					nearestObjectID.type = TRIANGLE;
					nearestObjectID.index = i;
					nearestPos = rayFrom + normalizedRayDir * distance;
					nearestNorm = obj.norm;
					*nearestMat = obj.material;
					isInside = false;
				}
			}
		}
	}
	return found;
}

bool isShaded(const Scene &scene, const RenderParams &params, const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const ObjectId &excludeObjectID) {
	ObjectId a;
	Vec3 b, c;
	Material *m;
	bool isInside;
	return findNearestObject(scene, params, rayFrom, normalizedRayDir, excludeObjectID, true, a, b, c, &m, isInside);
}

Color _renderPixel(const Scene &scene, const RenderParams &params, const Vec3 &rayFrom, const Vec3 &normalizedRayDir, ObjectId prevObjectID, int depth, float rIndex) {
	ObjectId objectID;
	Vec3 pos, norm;
	Material *m;
	bool isInside = false;
	if (!findNearestObject(scene, params, rayFrom, normalizedRayDir, prevObjectID, false, objectID, pos, norm, &m, isInside)) {
		return scene.bgColor;
	}

	Color c = scene.bgColor * m->ambientFactor;
	Vec3 reflectionDir = glm::normalize(glm::reflect(normalizedRayDir, norm));
	if (!m->refract) {
		for (const Light &light : scene.lights) {
			Vec3 lightDir;
			if (light.type == LT_POINT) {
				lightDir = glm::normalize(light.position - pos);
			}
			else if (light.type == LT_DIRECTIONAL) {
				lightDir = -light.position;
			}

			float s = glm::dot(norm, lightDir);
			if (s > 0.0f && !isShaded(scene, params, pos, lightDir, objectID)) {
				Color diffuse(s * light.intensity);
				c += diffuse * light.color * m->diffuseFactor;
			}

			float t = glm::dot(lightDir, reflectionDir);
			if (t > 0.0f && !isShaded(scene, params, pos, reflectionDir, objectID)) {
				Color specular = Color(powf(t, m->shininess) * light.intensity);
				c += specular * light.color * m->specularFactor;
			}
		}
	}

	if (depth < params.depthLimit) {
		c += _renderPixel(scene, params, pos, reflectionDir, objectID, depth + 1, rIndex) * m->reflectionFactor;
		if (m->refract) {
			float n = rIndex / m->refraction;
			Vec3 N = norm;
			if (isInside)
				N *= -1;
			float cosI = -glm::dot(N, normalizedRayDir);
			float cosT2 = 1.0f - n * n * (1.0f - cosI * cosI);
			if (cosT2 > 0.0f) {
				Vec3 refractionDir = n * normalizedRayDir + (n * cosI - sqrtf(cosT2)) * N;
				// For refraction we don't exclude current object
				c += _renderPixel(scene, params, pos + refractionDir * 1e-5f, refractionDir, {}, depth + 1, m->refraction) * m->refractionFactor;
			}
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
			Color c = _renderPixel(scene, params, scene.camera.position, glm::normalize(p - scene.camera.position), {}, 0, 1.0f);
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
