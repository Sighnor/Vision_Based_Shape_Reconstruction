#ifndef EXTRACTION_MATH
#define EXTRACTION_MATH

#include <iostream>
#include <assert.h>
#include <math.h>

#undef M_PI
#define M_PI 3.141592653589793f

inline float deg_rad(const float& deg) { return deg * M_PI / 180.0; }

template<typename T>
class vec2
{
    public:
        T x, y;
        
        vec2() : x(0.0), y(0.0) {}
        vec2(T _x, T _y) : x(_x), y(_y) {}

        friend std::ostream & operator << (std::ostream &output, vec2<T> &vec) 
        {
            output << vec.x << ", " << vec.y;
            return output;
        }
};

template<typename T>
inline vec2<T> operator + (const vec2<T>& v1, const vec2<T>& v2) 
{
    return vec2<T>(v1.x + v2.x, v1.y + v2.y);
}

template<typename T>
inline vec2<T> operator - (const vec2<T>& v1, const vec2<T>& v2) 
{
    return vec2<T>(v1.x - v2.x, v1.y - v2.y);
}

template<typename T>
inline vec2<T> operator * (float s, vec2<T> v)
{
    return (vec2(T(s * v.x), T(s * v.y)));
}

template<typename T>
inline float distance(vec2<T> v1, vec2<T> v2)
{
    return (sqrtf((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y)));
}

template<typename T>
class vec3
{
    public:
        T x, y, z;

        vec3() : x(0.0), y(0.0), z(0.0) {}
        vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

        friend std::ostream & operator << (std::ostream &output, vec3<T> &vec) 
        {
            output << vec.x << ", " << vec.y << ", " << vec.z;
            return output;
        }
};

template<typename T>
inline vec3<T> operator + (const vec3<T>& v1, const vec3<T>& v2) 
{
    return vec3<T>(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

template<typename T>
inline vec3<T> operator - (const vec3<T>& v1, const vec3<T>& v2) 
{
    return vec3<T>(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

template<typename T>
inline vec3<T> operator * (float s, const vec3<T>& v) 
{
    return vec3<T>(T(s * v.x), T(s * v.y), T(s * v.z));
}

template<typename T>
inline vec3<T> normalize(const vec3<T>& v) 
{
    float mag_2 = v.x * v.x + v.y * v.y + v.z * v.z;

    if (mag_2 > 0) 
    {
        float invMag = 1 / sqrtf(mag_2);
        return invMag * v;
    }

    return v;
}

template<typename T>
inline float dot(const vec3<T>& v1, const vec3<T>& v2)
{
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

template<typename T>
class mat3
{
    public:
        vec3<T> X_axis, Y_axis, Z_axis;

        mat3() : X_axis(vec3<T>()), Y_axis(vec3<T>()), Z_axis(vec3<T>()) {}
        mat3(vec3<T> _X_axis, vec3<T> _Y_axis, vec3<T> _Z_axis) : X_axis(_X_axis), Y_axis(_Y_axis), Z_axis(_Z_axis) {}

        inline vec3<T> operator * (const vec3<T>& v) 
        {
            return vec3<T>((X_axis.x * v.x + Y_axis.x * v.y + Z_axis.x * v.z),
                            (X_axis.y * v.x + Y_axis.y * v.y + Z_axis.y * v.z),    
                            (X_axis.z * v.x + Y_axis.z * v.y + Z_axis.z * v.z));
        }

        friend std::ostream & operator << (std::ostream &output, mat3<T> &mat) 
        {
            output << mat.X_axis.x << ", " << mat.Y_axis.x << ", " << mat.Z_axis.x << std::endl
                   << mat.X_axis.y << ", " << mat.Y_axis.y << ", " << mat.Z_axis.y << std::endl
                   << mat.X_axis.z << ", " << mat.Y_axis.z << ", " << mat.Z_axis.z;
            return output;
        }
};
// Taken from https://github.com/orangeduck/Motion-Matching
template<typename T>
struct array1d
{
    int size;
    T* data;
    
    array1d() : size(0), data(NULL) {}
    array1d(int _size) : array1d() { resize(_size);  }
    array1d(const array1d<T>& rhs) : array1d() { resize(rhs.size); memcpy(data, rhs.data, rhs.size * sizeof(T)); }
    
    array1d& operator=(const array1d<T>& rhs) { resize(rhs.size); memcpy(data, rhs.data, rhs.size * sizeof(T)); return *this; };

    inline T& operator()(int i) const { assert(i >= 0 && i < size); return data[i]; }
    
    friend std::ostream & operator << (std::ostream &output, array1d<T> &arr) 
    {
        for(int i = 0;i < arr.size;i++)
        {
            output << arr.data[i] << ' ';
        }
        return output;
    }

    void zero() { memset(data, 0, sizeof(T) * size); }
    void set(const T& x) { for (int i = 0; i < size; i++) { data[i] = x; } }
    
    void resize(int _size)
    {
        if (_size == 0 && size != 0)
        {
            free(data);
            data = NULL;
            size = 0;
        }
        else if (_size > 0 && size == 0)
        {
            data = (T*)malloc(_size * sizeof(T));
            size = _size;
            assert(data != NULL);
        }
        else if (_size > 0 && size > 0 && _size != size)
        {
            data = (T*)realloc(data, _size * sizeof(T));
            size = _size;
            assert(data != NULL);           
        }
    }
};

#endif