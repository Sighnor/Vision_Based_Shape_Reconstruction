#ifndef IMG_CORE
#define IMG_CORE

#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\types_c.h>
#include "curve.hpp"
#include "global.hpp"
#include "net.hpp"
#include "vec.hpp"

enum Curve
{
    Bezier  = 0,
    PID     = 1,
    Spine   = 2,
};

void Draw_Line(cv::Mat &img1, const std::vector<vec2> &points)
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

void Draw_Curve(cv::Mat &img1, const std::vector<vec2> &points, Curve curve_type)
{
    switch (curve_type)
    {
        case Bezier:
        {
            bezier_curve(img1, points);
            break;
        }
        case PID:
        {
            PID_curve(img1, points);
            break;
        }
        case Spine:
        {
            break;
        }
        default:
            break;
    }
}

void Draw_SOM(cv::Mat &img1, const vec2 contour_point, Net &net, int id)
{
    cv::Mat img2 = img1.clone();
    std::vector<vec2> points = net.Node_Data();

    // Draw_Line(img2, points);
    Draw_Curve(img2, points, PID);

    cv::circle(img2, cv::Point(points[id].x, points[id].y), 10, cv::Scalar(255, 0, 0)); 
    cv::circle(img2, cv::Point(contour_point.x, contour_point.y), 10, cv::Scalar(0, 255, 0)); 
    
    cv::imshow("SOM", img2);
    cv::waitKey(10);
}

cv::Mat Gaussian_filter(cv::Mat &img1, int m, int n, float k, float theta_2)
{
    int height = img1.rows;
    int width = img1.cols;
    cv::Mat img2 = img1.clone();
    std::vector<float> prt_core((m + 1) * (n + 1));

    Precompute_Core(prt_core, m, n, k, theta_2);

    #pragma omp parallel for
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
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

    cv::imshow("Guassian", img2);

    return img2;
}

std::vector<vec2> Canny(cv::Mat &img1)
{
    int height = img1.rows;
    int width = img1.cols;
    cv::Mat filter = Gaussian_filter(img1, 4, 4, 1.0, 1.5);
    cv::cvtColor(filter, filter, cv::COLOR_BGR2GRAY);
    cv::Mat G = filter.clone();
    std::vector<float> theta(height * width);
    std::vector<vec2> contour_points;

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

std::vector<vec2> Identify_Centerline(cv::Mat &img1, std::vector<vec2> &contour_points)
{
    int T = 12000;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    Net net;
    net.Read("../resourses/net.bin");
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

    std::vector<vec2> center_points = net.Node_Data();

    return center_points;
}

#endif
