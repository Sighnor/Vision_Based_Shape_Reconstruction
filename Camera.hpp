#include <stdio.h>
#include <fstream>
#include <string.h>
#include "Math.hpp"

class Ray
{
    public:
        vec3<float> ori;
        vec3<float> dir;

        Ray(vec3<float> _ori, vec3<float> _dir) : ori(_ori), dir(_dir) {}
};

inline vec3<float> Get_Intersection(const Ray &ray1, const Ray &ray2)
{
    float a = dot(ray1.dir, ray1.dir);
    float b = -dot(ray1.dir, ray2.dir);
    float c = -dot(ray1.dir, ray2.dir);
    float d = dot(ray2.dir, ray2.dir);
    float delta1 = dot((ray2.ori - ray1.ori), ray1.dir);
    float delta2 = dot((ray1.ori - ray2.ori), ray2.dir);

    float det = std::max(a * d - b * c, 0.001f);
    float det1 = delta1 * d - b * delta2;
    float det2 = a * delta2 - delta1 * c;

    float t1 = det1 / det;
    float t2 = det2 / det;

    vec3<float> point1 = ray1.ori + t1 * ray1.dir; 
    vec3<float> point2 = ray2.ori + t2 * ray2.dir; 

    return 0.5f * (point1 + point2); 
}

enum Side {LEFT, RIGHT};

class Camera
{
    public:
        int fov;
        int width;
        int height;
        float focus;
        float distance;
        vec3<float> position;
        mat3<float> orientation;

        Camera(int _fov, int _width, int _height, float _focus, float _distance, vec3<float> _position, mat3<float> _orientation) :
        fov(_fov), width(_width), height(_height), focus(_focus), distance(_distance), position(_position), orientation(_orientation) {}

        inline Ray Get_Ray(float x, float y, Side side);
        inline vec3<float> Rebuild(const vec2<float> &point1, const vec2<float> &point2);
};

inline Ray Camera::Get_Ray(float x, float y, Side side)
{
    float scale = tan(deg_rad(fov * 0.5));
    float imageAspectRatio = width / height;

    vec3<float> ori;
    vec3<float> dir;

    float _x = (2 * x / width - 1) * imageAspectRatio * scale;
    float _y = (1 - 2 * y / height) * scale;
    float _z = -1.f;

    switch (side)
    {
    case LEFT:
        ori = vec3<float>(-distance, 0.f, 0.f);
        dir = normalize(vec3<float>(_x, _y, _z));
        break;
    case RIGHT:
        ori = vec3<float>(distance, 0.f, 0.f);
        dir = normalize(vec3<float>(_x, _y, _z));
        break;
    default:
        break;
    }

    return Ray(ori, dir);
}

inline vec3<float> Camera::Rebuild(const vec2<float> &point1, const vec2<float> &point2)
{
    Ray l_ray = Get_Ray(point1.x, point1.y, LEFT);
    Ray r_ray = Get_Ray(point2.x, point2.y, RIGHT);
    std::cout << l_ray.dir << std::endl;
    vec3<float> cordiantes = Get_Intersection(l_ray, r_ray);
    return position + (orientation * cordiantes);
}