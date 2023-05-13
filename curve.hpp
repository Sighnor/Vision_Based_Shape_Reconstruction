#ifndef IMG_CURVE
#define IMG_CURVE

#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\types_c.h>
#include "vec.hpp"

void Draw_point(cv::Mat &img1, vec2 &point, std::vector<float> &core)
{
    for(int i = -1;i <= 1;i++)
    {
        for(int j = -1;j <= 1;j++)
        {
            img1.at<cv::Vec3b>(point.y + i, point.x + j)[0] = 0;
            img1.at<cv::Vec3b>(point.y + i, point.x + j)[1] = 0;
            img1.at<cv::Vec3b>(point.y + i, point.x + j)[2] = core[abs(i) * 2  + abs(j)] * 255;
        }
    }
}

vec2 recursive_bezier(const std::vector<vec2> &points, float t) 
{
    std::vector<vec2> sub_points;

    if(points.size() == 2)
    {
        return (1-t) * points[0] + t * points[1];
    }
    
    for(int i = 0;i < (points.size() -1);i++)
    {
       sub_points.push_back((1-t) * points[i] + t * points[i+1]);
    }

    return recursive_bezier(sub_points,t);
}

void bezier_curve(cv::Mat &img1, const std::vector<vec2> &points) 
{
    std::vector<float> prt_core((1 + 1) * (1 + 1));
    Precompute_Core(prt_core, 1, 1, 1.0, 1.5);

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(points,t);

        Draw_point(img1, point, prt_core);
    }
}

void pos_pid_control(
                    vec2 &x,
                    vec2 &vel,
                    vec2 &acc,
                    vec2 x_goal,
                    vec2 p_const, 
                    vec2 i_const, 
                    vec2 d_const, 
                    float k, 
                    float dt)
{
    static vec2 pos_ek_0 = vec2(0.f);
    static vec2 pos_ek_1 = vec2(0.f);
    static vec2 pos_ek_2 = vec2(0.f);

    pos_ek_2 = pos_ek_1;
    pos_ek_1 = pos_ek_0;
    pos_ek_0 = x_goal - x;

    acc = k * ((p_const * (pos_ek_0 - pos_ek_1) + i_const * pos_ek_0 - d_const * (pos_ek_0 - 2 * pos_ek_1 + pos_ek_2)));
    vel = vel + acc * dt;
    x = x + vel * dt;
}

void PID_curve(cv::Mat &img1, const std::vector<vec2> &points)
{
    std::vector<float> prt_core((1 + 1) * (1 + 1));
    Precompute_Core(prt_core, 1, 1, 1.0, 1.5);

    vec2 x = points[0];
    vec2 vel = vec2(0.f);
    vec2 acc = vec2(0.f);

    int sample_num = 300;

    for(float t = 0; t < 1.f; t += 1.f / sample_num)
    {
        float i = t * (points.size() - 1);
        int i0 = floor(i);
        int i1 = ceil(i);
        vec2 point = lerp(points[i0], points[i1], i - i0);
        // vec2 point = points[i1];

        pos_pid_control(
                        x,
                        vel,
                        acc,
                        point,
                        vec2(1.f, 1.f), 
                        vec2(0.05f), 
                        vec2(0.2f), 
                        200.f, 
                        1.f / 60.f);

        Draw_point(img1, x, prt_core);
    }
}

#endif