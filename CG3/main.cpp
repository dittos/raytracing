#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#include <glut.h>
#else
#include <OpenGL/GL.h>
#include <GLUT/GLUT.h>
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
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define ENABLE_OCTREE

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

const int OCTREE_DEPTH = 5;
const int DEPTH_LIMIT = 4;
int windowWidth;
int windowHeight;
Color *pixels = nullptr;
std::vector<Sphere> spheres;
std::vector<Triangle> triangles;
std::vector<Light> lights;
OctreeNode octreeRoot;
Camera camera;
Color bgColor;

BoundingBox extend(const BoundingBox &bbox, float d) {
	BoundingBox e = {
			{ bbox.min.x - d, bbox.min.y - d, bbox.min.z - d },
			{ bbox.max.x + d, bbox.max.y + d, bbox.max.z + d }
	};
	return e;
}

BoundingBox get_bbox(const Triangle &obj) {
    BoundingBox b = {
        {
            std::min({obj.vertex[0].x, obj.vertex[1].x, obj.vertex[2].x}),
            std::min({obj.vertex[0].y, obj.vertex[1].y, obj.vertex[2].y}),
            std::min({obj.vertex[0].z, obj.vertex[1].z, obj.vertex[2].z})
        },
        {
            std::max({obj.vertex[0].x, obj.vertex[1].x, obj.vertex[2].x}),
            std::max({obj.vertex[0].y, obj.vertex[1].y, obj.vertex[2].y}),
            std::max({obj.vertex[0].z, obj.vertex[1].z, obj.vertex[2].z})
        }
    };
    return extend(b, 1e-2f);
}

bool overlaps(const BoundingBox &bbox, const BoundingBox &b) {
    return !((b.max.x < bbox.min.x)
      || (b.max.y < bbox.min.y)
      || (b.max.z < bbox.min.z)
      || (b.min.x > bbox.max.x)
      || (b.min.y > bbox.max.y)
      || (b.min.z > bbox.max.z));
}

void splitOctreeNode(OctreeNode *node, int depth) {
    BoundingBox b = node->bounds;
    Vec3 center = (b.min + b.max) / 2.0f;
    BoundingBox bboxes[8] = {
        {b.min, center},
        {{center.x, b.min.y, b.min.z}, {b.max.x, center.y, center.z}},
        {{center.x, b.min.y, center.z}, {b.max.x, center.y, b.max.z}},
        {{b.min.x, b.min.y, center.z}, {center.x, center.y, b.max.z}},
        {{b.min.x, center.y, b.min.z}, {center.x, b.max.y, center.z}},
        {{center.x, center.y, b.min.z}, {b.max.x, b.max.y, center.z}},
        {center, b.max},
        {{b.min.x, center.y, center.z}, {center.x, b.max.y, b.max.z}}
    };
    for (int j = 0; j < 8; j++) {
        OctreeNode *subnode = new OctreeNode;
		subnode->leaf = true;
		subnode->bounds = extend(bboxes[j], 1e-2f);
        for (int i : node->objects) {
			if (overlaps(subnode->bounds, get_bbox(triangles[i]))) {
				subnode->objects.push_back(i);
            }
        }
		if (subnode->objects.size() == 0) {
			delete subnode;
			subnode = nullptr;
		}
		else {
			node->leaf = false;
			if (depth < OCTREE_DEPTH)
				splitOctreeNode(subnode, depth + 1);
		}
        node->subnodes[j] = subnode;
    }
}

void buildOctree() {
    float size = 10;
    octreeRoot.bounds.min = {-size, -size, -size};
    octreeRoot.bounds.max = {size, size, size};
    for (int i = 0; i < triangles.size(); i++)
        octreeRoot.objects.push_back(i);
	octreeRoot.leaf = true;
    splitOctreeNode(&octreeRoot, 0);
}

bool intersectRayPlane(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const Vec3 &planeNormal, float planeD, Vec3 &p) {
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

bool intersectBboxRay(const BoundingBox &bbox, const Vec3 &rayFrom, const Vec3 &normalizedRayDir) {
    if (bbox.min.x < rayFrom.x && rayFrom.x < bbox.max.x &&
		bbox.min.y < rayFrom.y && rayFrom.y < bbox.max.y &&
		bbox.min.z < rayFrom.z && rayFrom.z < bbox.max.z)
        // bbox contains rayFrom
        return true;
    
    Vec3 p;
    
    if(intersectRayPlane(rayFrom, normalizedRayDir, Vec3({0, 0, 1}), bbox.min.z, p)
       && (p.x > bbox.min.x) && (p.x < bbox.max.x)
       && (p.y > bbox.min.y) && (p.y < bbox.max.y)) {
        return true;
    }
    
    if(intersectRayPlane(rayFrom, normalizedRayDir, Vec3({0, 0, 1}), bbox.max.z, p)
       && (p.x > bbox.min.x) && (p.x < bbox.max.x)
       && (p.y > bbox.min.y) && (p.y < bbox.max.y)) {
        return true;
    }
    
    if(intersectRayPlane(rayFrom, normalizedRayDir, Vec3({0, 1, 0}), bbox.min.y, p)
       && (p.x > bbox.min.x) && (p.x < bbox.max.x)
       && (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
        return true;
    }
    
    if(intersectRayPlane(rayFrom, normalizedRayDir, Vec3({0, 1, 0}), bbox.max.y, p)
       && (p.x > bbox.min.x) && (p.x < bbox.max.x)
       && (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
        return true;
    }
    
    if(intersectRayPlane(rayFrom, normalizedRayDir, Vec3({1, 0, 0}), bbox.min.z, p)
       && (p.y > bbox.min.y) && (p.y < bbox.max.y)
       && (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
        return true;
    }
    
    if(intersectRayPlane(rayFrom, normalizedRayDir, Vec3({1, 0, 0}), bbox.max.z, p)
       && (p.y > bbox.min.y) && (p.y < bbox.max.y)
       && (p.z > bbox.min.z) && (p.z < bbox.max.z)) {
        return true;
    }
    
    return false;
}

bool findNode(OctreeNode *node, const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const int excludeId, int &nearestId, float &nearestDist) {
	bool found = false;
	if (!node->leaf) {
		for (OctreeNode *subnode : node->subnodes) {
			// Early pruning!
			if (subnode == nullptr || !intersectBboxRay(subnode->bounds, rayFrom, normalizedRayDir))
				continue;

			if (findNode(subnode, rayFrom, normalizedRayDir, excludeId, nearestId, nearestDist))
				found = true;
		}
	}
	else {
		Vec3 baryPos;
		for (int i : node->objects) {
			if (i == excludeId) continue;
			const Triangle &obj = triangles[i];
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

bool findNearestObject(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const ObjectId excludeObjectID, bool excludeTransparentMat, ObjectId &nearestObjectID, Vec3 &nearestPos, Vec3 &nearestNorm, Material **nearestMat, bool &isInside) {
    bool found = false;
    float nearestDist = std::numeric_limits<float>::max();
    for (int i = 0; i < spheres.size(); i++) {
        if (excludeObjectID.type == SPHERE && i == excludeObjectID.index)
            continue;
        const Sphere &obj = spheres[i];
        if (excludeTransparentMat && obj.material->refract)
            continue;
        Vec3 pos, norm;
        if (glm::intersectRaySphere(rayFrom, normalizedRayDir, obj.center, obj.radius, pos, norm)) {
            float distance = glm::distance(rayFrom, pos);
            if (distance < nearestDist) {
                found = true;
                nearestDist = distance;
                nearestObjectID.type = SPHERE;
                nearestObjectID.index = i;
                nearestPos = pos;
                nearestNorm = norm;
                *nearestMat = obj.material;
                isInside = glm::distance(rayFrom, obj.center) < obj.radius;
            }
        }
    }
#ifdef ENABLE_OCTREE
    if (findNode(&octreeRoot, rayFrom, normalizedRayDir, excludeObjectID.type == TRIANGLE ? excludeObjectID.index : -1, nearestObjectID.index, nearestDist)) {
        found = true;
        nearestObjectID.type = TRIANGLE;
        nearestPos = rayFrom + normalizedRayDir * nearestDist;
        nearestNorm = triangles[nearestObjectID.index].norm;
        *nearestMat = triangles[nearestObjectID.index].material;
        isInside = false;
    }
#else
    for (int i = 0; i < triangles.size(); i++) {
        if (excludeObjectID.type == TRIANGLE && i == excludeObjectID.index)
            continue;
        const Triangle &obj = triangles[i];
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
                nearestPos = rayFrom + normalizedRayDir * baryPos.z;
                nearestNorm = obj.norm;
                *nearestMat = obj.material;
                isInside = false;
            }
        }
    }
#endif
    return found;
}

bool isShaded(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const ObjectId &excludeObjectID) {
    ObjectId a;
    Vec3 b, c;
    Material *m;
    bool isInside;
    return findNearestObject(rayFrom, normalizedRayDir, excludeObjectID, true, a, b, c, &m, isInside);
}

Color _renderPixel(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, ObjectId prevObjectID, int depth, float rIndex) {
    ObjectId objectID;
    Vec3 pos, norm;
    Material *m;
    bool isInside = false;
    if (!findNearestObject(rayFrom, normalizedRayDir, prevObjectID, false, objectID, pos, norm, &m, isInside)) {
        return bgColor;
    }
    
    Color c = bgColor * m->ambientFactor;
    Vec3 reflectionDir = glm::normalize(glm::reflect(normalizedRayDir, norm));
    if (!m->refract) {
        for (const Light &light : lights) {
            Vec3 lightDir;
			if (light.type == LT_POINT) {
                lightDir = glm::normalize(light.position - pos);
			}
			else if (light.type == LT_DIRECTIONAL) {
                lightDir = -light.position;
            }

            float s = glm::dot(norm, lightDir);
            if (s > 0.0f && !isShaded(pos, lightDir, objectID)) {
                Color diffuse(s * light.intensity);
                c += diffuse * light.color * m->diffuseFactor;
            }
            
            float t = glm::dot(lightDir, reflectionDir);
            if (t > 0.0f && !isShaded(pos, reflectionDir, objectID)) {
                Color specular = Color(powf(t, m->shininess) * light.intensity);
                c += specular * light.color * m->specularFactor;
            }
        }
    }

    if (depth < DEPTH_LIMIT) {
        c += _renderPixel(pos, reflectionDir, objectID, depth + 1, rIndex) * m->reflectionFactor;
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
                c += _renderPixel(pos + refractionDir * 1e-5f, refractionDir, {}, depth + 1, m->refraction) * m->refractionFactor;
            }
        }
    }
    return glm::clamp(c, 0.0f, 1.0f);
}

Color renderPixel(const Vec3 &p) {
    return _renderPixel(camera.position, glm::normalize(p - camera.position), {}, 0, 1.0f);
}

void _render(Color *pixels, int width, int height, int ntasks, int taskid, const Mat4 &proj, const glm::vec4 &viewport) {
    Mat4 model;
    for (int y = taskid; y < height; y += ntasks) {
        for (int x = 0; x < width; x++) {
            Vec3 win = {x, y, 0};
            Vec3 p = glm::unProject(win, model, proj, viewport);
            pixels[y * width + x] = renderPixel(p);
        }
    }
}

void render(Color *pixels, int width, int height) {
    time_t start = time(NULL);
    Mat4 proj = glm::perspective(camera.fovy * 3.14159265358979323846f / 180.0f, camera.aspect, camera.zNear, camera.zFar) *
        glm::lookAt(camera.position, camera.at, camera.up);
    glm::vec4 viewport(0, 0, width, height);
    const int ntasks = 4;
    std::vector<std::future<void>> tasks;
    for (int i = 0; i < ntasks; i++) {
        tasks.push_back(std::async(std::launch::async, _render, pixels, width, height, ntasks, i, proj, viewport));
    }
    for (int i = 0; i < tasks.size(); i++) {
        tasks[i].get();
    }
    time_t end = time(NULL);
    std::cout << "time: " << end - start << std::endl;
}

void reshape(int width, int height) {
    windowWidth = width;
    windowHeight = height;
    glViewport(0, 0, width, height);
    gluOrtho2D(0, width, 0, height);
    if (pixels != nullptr) {
        delete [] pixels;
    }
    pixels = new Color[width * height];
    camera.aspect = (float)width / height;
}

void display(void) {
    render(pixels, windowWidth, windowHeight);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_POINTS);
    for (int y = 0; y < windowHeight; y++) {
        for (int x = 0; x < windowWidth; x++) {
            Color c = pixels[y * windowWidth + x];
            glColor3f(c.r, c.g, c.b);
            glVertex2i(x, y);
        }
    }
    glEnd();
    glutSwapBuffers();
}

Triangle make_triangle(Vec3 v0, Vec3 v1, Vec3 v2, Material *material) {
    Triangle t {{v0, v1, v2}, glm::normalize(glm::cross(v1 - v0, v2 - v0)), material};
    return t;
}

void readModel(std::string path, float scaleFactor, Material *material) {
    std::ifstream f(path);
    std::string line;
    std::vector<Vec3> vs;
    
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string type;
        ss >> type;
        if (type == "v") {
            Vec3 v;
            ss >> v.x >> v.y >> v.z;
            vs.push_back(v * scaleFactor);
        } else if (type == "f") {
            std::string v_str;
            std::vector<int> f;
            while (ss >> v_str) {
                int v = atoi(v_str.c_str());
                if (v < 0)
                    v += vs.size();
                else
                    v--;
                f.push_back(v);
            }
            for (int i = 0; i < f.size() - 2; i++) {
                triangles.push_back(make_triangle(vs[f[i]], vs[f[i + 1]], vs[f[i + 2]], material));
            }
        }
    }
    
    std::cout << "vertex: " << vs.size() << std::endl;
}

int main(int argc, char *argv[]) {
    Material copper = {{0.329412, 0.223529, 0.027451},
        {0.780392, 0.568627, 0.113725},
        {0.992157, 0.941176, 0.807843},
        27.8974,
        0.2};
    Material chrome = {{0.25, 0.25, 0.25},
        {0.4, 0.4, 0.4},
        {0.774597, 0.774597, 0.774597},
        76.8,
        0.3};
    Material glass = {{0.25, 0.25, 0.25},
        {0.4, 0.4, 0.4},
        {0.774597, 0.774597, 0.774597},
        76.8,
        0.0};
    glass.refract = true;
    glass.refraction = 1.3f;
    glass.refractionFactor = 1.0f;
    spheres.push_back({{-0.02, 0.2, 0.55}, 0.02, &glass});
//    spheres.push_back({{-0.45, 0.1, -0.25}, 0.05, &copper});
    spheres.push_back({{0.25, 0.1, 0.0}, 0.1, &chrome});
//    spheres.push_back({{-0.2, 0.1, 0.0}, 0.05, &chrome});
    
    float w = 1.0, front = 1.0, back = -1.0, h = 1.0, y = -0.01f;
    triangles.push_back(make_triangle({-w, y, back}, {-w, y, front}, {w, y, back}, &chrome));
    triangles.push_back(make_triangle({w, y, back}, {-w, y, front}, {w, y, front}, &chrome));
//    triangles.push_back(make_triangle({-w, h, back}, {-w, 0, back}, {w, 0, back}, &copper));
//    triangles.push_back(make_triangle({w, h, back}, {-w, h, back}, {w, 0, back}, &copper));
//    triangles.push_back(make_triangle({-w, h, back}, {-w, 0, front}, {-w, 0, back}, &copper));
//    triangles.push_back(make_triangle({-w, h, front}, {-w, 0, front}, {-w, h, back}, &copper));
    readModel("2009210107_3.obj", 0.5, &copper);
#ifdef ENABLE_OCTREE
    buildOctree();
    std::cout << "built octree" << std::endl;
#endif

    camera.position = {0.0, 0.2, 0.7};
    camera.at = {0.0, 0.1, 0.0};
    camera.up = {0.0, 1.0, 0.0};
    camera.zNear = 0.01;
    camera.zFar = 10.0;
    camera.fovy = 60;
    bgColor = {0.0, 0.0, 0.0};
//    lights.push_back({LT_POINT, {0.0, 0.0, 1.0}, 1.0, {1.0, 0.0, 0.0}});
//    lights.push_back({LT_POINT, {0.5, 0.5, 0.5}, 0.5, {1.0, 0.0, 0.0}});
    lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({0.5f, -0.5f, 1.0f})), 1.0, {0.0, 1.0, 1.0}});
    //lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({0.5f, -0.5f, -1.0f})), 1.0, {1.0, 0.0, 1.0}});
    //lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({-0.5f, -0.5f, 0.0f})), 1.0, {1.0, 1.0, 0.0}});
	lights.push_back({LT_DIRECTIONAL, glm::normalize(Vec3({ -0.5f, -0.5f, -1.0f })), 2.0, { 1.0, 1.0, 1.0 } });

    const int width = 1280, height = 720;
    camera.aspect = (float)width / height;
    Color *data = new Color[width * height];
    render(data, width, height);
    unsigned char *bytedata = new unsigned char[width * height * 3]; // RGB
    int i = 0;
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            Color c = data[y * width + x];
            bytedata[i++] = 255 * c.r; // r
            bytedata[i++] = 255 * c.g; // g
            bytedata[i++] = 255 * c.b; // b
        }
    }
    stbi_write_png("out.png", width, height, 3, bytedata, 0);

	/*
	windowWidth = width;
	windowHeight = height;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("2009210107_Term");
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutMainLoop();
	*/
    return 0;
}
