#include <OpenGL/GL.h>
#include <GLUT/GLUT.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <future>
#include "glm/vec3.hpp"
#include "glm/mat4x4.hpp"
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

struct PointLight {
    Vec3 position;
    float intensity;
};

const int DEPTH_LIMIT = 3;
int windowWidth;
int windowHeight;
Color *pixels = nullptr;
std::vector<Sphere> spheres;
std::vector<Triangle> triangles;
PointLight light;
Camera camera;
Color bgColor;

bool findNearestObject(const Vec3 rayFrom, const Vec3 normalizedRayDir, const ObjectId excludeObjectID, ObjectId &nearestObjectID, Vec3 &nearestPos, Vec3 &nearestNorm, Material **nearestMat) {
    bool found = false;
    float nearestDist = std::numeric_limits<float>::max();
    for (int i = 0; i < spheres.size(); i++) {
        if (excludeObjectID.type == SPHERE && i == excludeObjectID.index)
            continue;
        const Sphere &obj = spheres[i];
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
            }
        }
    }
    for (int i = 0; i < triangles.size(); i++) {
        if (excludeObjectID.type == TRIANGLE && i == excludeObjectID.index)
            continue;
        const Triangle &obj = triangles[i];
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
            }
        }
    }
    return found;
}

bool isShaded(const Vec3 &rayFrom, const Vec3 &normalizedRayDir, const ObjectId &excludeObjectID) {
    ObjectId a;
    Vec3 b, c;
    Material *m;
    return findNearestObject(rayFrom, normalizedRayDir, excludeObjectID, a, b, c, &m);
}

Color _renderPixel(Vec3 rayFrom, Vec3 normalizedRayDir, ObjectId prevObjectID, int depth) {
    ObjectId objectID;
    Vec3 pos, norm;
    Material *m;
    if (!findNearestObject(rayFrom, normalizedRayDir, prevObjectID, objectID, pos, norm, &m)) {
        return bgColor;
    }
    
    Color c = bgColor * m->ambientFactor;
    
    Vec3 lightDir = glm::normalize(light.position - pos);
    if (!isShaded(pos, lightDir, objectID)) {
        Color diffuse(fabs(glm::dot(norm, lightDir)) * light.intensity);
        c += diffuse * m->diffuseFactor;
    }
    
    Vec3 reflectionDir = glm::normalize(glm::reflect(pos - rayFrom, norm));
    float s = glm::dot(lightDir, reflectionDir);
    if (s > 0.0f && !isShaded(pos, reflectionDir, objectID)) {
        Color specular = Color(powf(s, m->shininess) * light.intensity);
        c += specular * m->specularFactor;
    }

    if (depth < DEPTH_LIMIT) {
        c += _renderPixel(pos, reflectionDir, objectID, depth + 1) * m->reflectionFactor;
    }
    return c;
}

Color renderPixel(const Vec3 &p) {
    return _renderPixel(camera.position, glm::normalize(p - camera.position), {}, 0);
}

void _render(Color *pixels, int width, int starty, int endy, Mat4 proj, glm::vec4 viewport) {
    Mat4 model;
    for (int y = starty; y < endy; y++) {
        for (int x = 0; x < width; x++) {
            Vec3 win = {x, y, 0};
            Vec3 p = glm::unProject(win, model, proj, viewport);
            pixels[y * width + x] = renderPixel(p);
        }
    }
}

void render(Color *pixels, int width, int height) {
    Mat4 proj = glm::perspective(camera.fovy * 3.14159265358979323846f / 180.0f, camera.aspect, camera.zNear, camera.zFar);
    glm::vec4 viewport(0, 0, width, height);
    const int ntasks = 4;
    std::vector<std::future<void>> tasks;
    for (int y = 0; y < height; y += height / ntasks) {
        tasks.push_back(std::async(std::launch::async, _render, pixels, width, y, y + height / ntasks, proj, viewport));
    }
    for (int i = 0; i < tasks.size(); i++) {
        tasks[i].get();
    }
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
        0.2};
    spheres.push_back({{-0.1, 0.1, 0.0}, 0.1, &copper});
    spheres.push_back({{0.15, 0.05, 0.0}, 0.05, &chrome});
//    spheres.push_back({{0.15, 0.1, -0.5}, 0.05, &chrome});
    
    float w = 0.5, front = 1, back = -1;
    triangles.push_back(make_triangle({-w, 0, back}, {-w, 0, front}, {w, 0, back}, &chrome));
    triangles.push_back(make_triangle({w, 0, back}, {-w, 0, front}, {w, 0, front}, &chrome));

    camera.position = {0.0, 1.5, 7.0};
    camera.at = {0.0, 0.5, 0.0};
    camera.up = {0.0, 1.0, 0.0};
    camera.zNear = 0.4;
    camera.zFar = 1000;
    camera.fovy = 90;
    bgColor = {0.0, 0.0, 0.0};
    light.position = {1.0, 2.0, 5.0};
    light.intensity = 2.0;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(windowWidth = 1280, windowHeight = 720);
    glutCreateWindow("2009210107_Term");
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}
