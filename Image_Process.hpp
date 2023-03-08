#pragma once

#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\types_c.h>
#include "Net.hpp"
#include "Math.hpp"
//高斯核
static inline float Core(int s, int t, float k, float theta2)
{
    float distance2 = s * s + t * t;
    return k * exp(-distance2 / (2 * theta2));
}

static inline void Precompute_Core(std::vector<float> &prt_core, int m, int n, float k, float theta2)
{
    for (int a = 0; a <= m; a++)
    {
        for (int b = 0; b <= n; b++)
        {
            prt_core[a * (n + 1) + b] = Core(a, b, k, theta2);//预计算高斯核
        }
    }
}

static inline vec2<float> recursive_bezier(const std::vector<vec2<float>> &points, float t) 
{
    std::vector<vec2<float>> sub_points;

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

static inline void bezier(cv::Mat &img1, const std::vector<vec2<float>> &points) 
{
    std::vector<float> prt_core((1 + 1) * (1 + 1));
    Precompute_Core(prt_core, 1, 1, 1.0, 1.5);

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(points,t);

        for(int i = -1;i <= 1;i++)
        {
            for(int j = -1;j <= 1;j++)
            {
                img1.at<cv::Vec3b>(point.y + i, point.x + j)[0] = 0;
                img1.at<cv::Vec3b>(point.y + i, point.x + j)[1] = 0;
                img1.at<cv::Vec3b>(point.y + i, point.x + j)[2] = prt_core[abs(i) * 2  + abs(j)] * 255;
            }
        }
    }
}

static inline void Draw_Line(cv::Mat &img1, const std::vector<vec2<float>> &points)
{
    for(int i = 0;i < points.size() - 1;i++)
    {
        cv::line(img1, 
                 cv::Point(points[i].x, points[i].y), 
                 cv::Point(points[i + 1].x, points[i + 1].y),   
                 cv::Scalar(0, 0, 255),
                 2);
    }
}

static inline void Draw_Curve(cv::Mat &img1, const std::vector<vec2<float>> &points)
{
    bezier(img1, points);
}

static inline void Draw_SOM(cv::Mat &img1, const vec2<float> contour_point, Net &net, int id)
{
    cv::Mat img2 = img1.clone();
    std::vector<vec2<float>> points = net.Node_Data();

    Draw_Line(img2, points);
    Draw_Curve(img2, points);

    cv::circle(img2, cv::Point(points[id].x, points[id].y), 10, cv::Scalar(255, 0, 0)); 
    cv::circle(img2, cv::Point(contour_point.x, contour_point.y), 10, cv::Scalar(0, 255, 0)); 
    
    cv::imshow("SOM", img2);
    cv::waitKey(10);
}
//高斯滤波
static inline cv::Mat Gaussian_filter(cv::Mat &img1, int m, int n, float k, float theta2)
{
    int height = img1.rows;
    int width = img1.cols;
    cv::Mat img2 = img1.clone();
    std::vector<float> prt_core((m + 1) * (n + 1));

    Precompute_Core(prt_core, m, n, k, theta2);

    #pragma omp parallel for
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            //边界判断，防止溢出
            int neg_yRadius = -std::min(y, m);
            int neg_xRadius = -std::min(x, n);
            int pos_yRadius = std::min(height - 1 - y, m);
            int pos_xRadius = std::min(width - 1 - x, n);
            float B = 0.0f;
            float G = 0.0f;
            float R = 0.0f;
            float sum_Core = 0.0f;
            for (int a = neg_yRadius; a <= pos_yRadius; a++)
            {
                for (int b = neg_xRadius; b <= pos_xRadius; b++)
                {
                    float temp_core = prt_core[abs(a) * (n + 1) + abs(b)];
                    B += img1.at<cv::Vec3b>(y + a, x + b)[0] * temp_core;
                    G += img1.at<cv::Vec3b>(y + a, x + b)[1] * temp_core;
                    R += img1.at<cv::Vec3b>(y + a, x + b)[2] * temp_core;
                    sum_Core += temp_core;
                }
            }
            img2.at<cv::Vec3b>(y, x)[0] = B / sum_Core;
            img2.at<cv::Vec3b>(y, x)[1] = G / sum_Core;
            img2.at<cv::Vec3b>(y, x)[2] = R / sum_Core;
        }
    }
    return img2;
}

static inline std::vector<vec2<float>> Canny(cv::Mat &img1)
{
    int height = img1.rows;
    int width = img1.cols;
    cv::Mat filter = Gaussian_filter(img1, 4, 4, 1.0, 1.5);
    cv::cvtColor(filter, filter, cv::COLOR_BGR2GRAY);
    cv::Mat G = filter.clone();
    std::vector<float> theta(height * width);
    std::vector<vec2<float>> contour_points;

    #pragma omp parallel for
    for (int y = 1; y < height - 1; y++)
    {
        for (int x = 1; x < width - 1; x++)
        {
            uchar* neg_y = filter.ptr<uchar>(y - 1);
            uchar* pos_y = filter.ptr<uchar>(y + 1);
            uchar* mid_y = filter.ptr<uchar>(y);
            for (int a = -1; a <= 1; a++)
            {
                for (int b = -1; b <= 1; b++)
                {
                    float Gy = 0.333f * (pos_y[x - 1] + pos_y[x] + pos_y[x + 1] - neg_y[x - 1] - neg_y[x] - neg_y[x + 1]);
                    float Gx = 0.333f * (pos_y[x + 1] + mid_y[x + 1] + neg_y[x + 1] - pos_y[x - 1] - mid_y[x - 1] - neg_y[x - 1]);
                    G.ptr<uchar>(y)[x] = sqrtf(Gy * Gy + Gx * Gx);
                    theta[y * width + x] = Gx / Gy;
                }
            }
        }
    }

    for (int y = 1; y < height - 1; y++)
    {
        for (int x = 1; x < width - 1; x++)
        {
            filter.ptr<uchar>(y)[x] = G.ptr<uchar>(y)[x];
            if(theta[y * width + x] >= 2.414 || theta[y * width + x] <= -2.414)
            {
                if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y)[x - 1])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                }
                else if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y)[x + 1])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                }
            }
            else if(theta[y * width + x] <= 0.414 && theta[y * width + x] >= -0.414)
            {
                if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y - 1)[x])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                    }
                else if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y + 1)[x])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                    }
            }
            else if(theta[y * width + x] > 0)
            {
                if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y - 1)[x - 1])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                }
                else if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y + 1)[x + 1])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                }
            }
            else
            {
                if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y - 1)[x + 1])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                }
                else if(filter.ptr<uchar>(y)[x] < G.ptr<uchar>(y + 1)[x - 1])
                {
                    filter.ptr<uchar>(y)[x] = 0.0;
                }
            }
            if(filter.ptr<uchar>(y)[x] > 0.0)
            {
                contour_points.push_back(vec2(float(x), float(y)));
            }
        }
    }

    cv::threshold(filter, filter, 10, 255, CV_THRESH_BINARY);

    cv::imshow("Canny", filter);

    return contour_points;
}

static inline std::vector<vec2<float>> Identify_Centerline(cv::Mat &img1, std::vector<vec2<float>> &contour_points)
{
    int T = 12000;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    Net net;
    net.Read("net.bin");
	net.SOM_Init(img1.rows, img1.cols);

    for(int t = 0;t < 2 * T;t++)
    {
        int cord_id = dist(rng) * contour_points.size();
        int id = net.SOM_Find_Min(contour_points[cord_id]);
        float alpha = 0.01 / (float(1) + float(t) / float(T));
        float sigma = t < T? 1.0 : 1.0;
        if(t % 60 == 0)
        {
            Draw_SOM(img1, contour_points[cord_id], net, id);
        }

        net.SOM_Update(contour_points[cord_id], id, alpha, sigma);
    }

    std::vector<vec2<float>> center_points = net.Node_Data();

    return center_points;
}