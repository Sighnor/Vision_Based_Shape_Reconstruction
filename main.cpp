#include "camera.hpp"
#include "image_process.hpp"
#include "mat.hpp"
#include "quat.hpp"

int main(int argc, char** argv)
{
    int online;

    int min_H = 0; 
    int max_H = 255; 
    int min_S = 0; 
    int max_S = 255; 
    int min_V = 0; 
    int max_V = 255; 

    std::cout << "online?" << std::endl;
    std::cin >> online;

    switch (online)
    {
        case 0:
            {
                int fov = 90;
                int width = 540;
                int height = 540;
                float focus = 0.05f;
                float distance = 1.f;
                vec3 position(8.f, 2.f, 0.f);
                mat3 orientation(vec3(0.f, 0.f, -1.f), vec3(0.f, 1.f, 0.f), vec3(1.f, 0.f, 0.f));

                Camera camera(fov, width, height, focus, distance, position, orientation);

                array2d<float> rotations(5000, 4);
                array2d<quat> local_quaternions(5000, 10);
                array2d<float> local_positions(5000, 10);

                Net net;
                // net.Write("../resources/net.bin", 10);
                net.Read("../resources/net.bin");

                int t = 0;
                int key = 0;

                min_H = 35;
                max_H = 77;

                while(t < 5000 && key != 27)
                {
                    char left_name[60];
                    char right_name[60];
                    sprintf(left_name, "../resources/img/zleft%d.png", t);
                    sprintf(right_name, "../resources/img/zright%d.png", t);

                    cv::Mat l_img;
                    l_img = cv::imread(left_name);

                    cv::Mat r_img;
                    r_img = cv::imread(right_name);

                    std::vector<vec2> l_contour_points = Canny(l_img, "Canny_left", min_H, max_H, min_S, max_S, min_V, max_V);
                    std::vector<vec2> r_contour_points = Canny(r_img, "Canny_right", min_H, max_H, min_S, max_S, min_V, max_V);

                    if(l_contour_points.size() > 0 && r_contour_points.size() > 0)
                    {
                        std::vector<vec2> l_center_points = Identify_Centerline(l_img, net, l_contour_points, "SOM_left");
                        std::vector<vec2> r_center_points = Identify_Centerline(r_img, net, r_contour_points, "SOM_right");

                        // std::cout << "begin!" << std::endl << std::endl;

                        array1d<vec3> positions(10);
                        array1d<quat> global_quaternions(10);

                        for(int i = 0; i < 10; i++)
                        {
                            positions(i) = camera.Rebuild(l_center_points[i], r_center_points[i]);
                        }

                        for(int i = 1; i < 10; i++)
                        {
                            vec3 v0 = normalize(vec3(0.f, 1.f, 0.f));
                            vec3 v1 = normalize(positions(i) - positions(i - 1));
                            vec3 axis = normalize(cross(v0, v1));
                            float phi = rad_to_deg(acos(dot(v0, v1)));
                            global_quaternions(i - 1) = quat(phi, axis);
                        }

                        local_quaternions(t, 0) = global_quaternions(0);
                        local_positions(t, 0) = rad_to_deg(PI / 4.f * sin(deg_to_rad(0.63f * t)));
                        for(int i = 1; i < 9; i++)
                        {
                            local_quaternions(t, i) = inv_quat(global_quaternions(i - 1)) * global_quaternions(i);
                            vec3 v0 = normalize(inv_quat(quat(local_positions(t, 0), vec3(0.f, 1.f, 0.f))) * (positions(i) - positions(i - 1)));
                            vec3 v1 = normalize(inv_quat(quat(local_positions(t, 0), vec3(0.f, 1.f, 0.f))) * (positions(i + 1) - positions(i)));
                            print(normalize(cross(v0, v1)));
                            local_positions(t, i) = rad_to_deg(acos(dot(v0, v1)));
                            if(cross(v0, v1).x > 0.8f)
                            {
                                local_positions(t, i) = rad_to_deg(acos(dot(v0, v1)));
                            }
                            else
                            {
                                local_positions(t, i) = - rad_to_deg(acos(dot(v0, v1)));
                            }
                        }
                        local_quaternions(t, 9) = quat(1.f, 0.f, 0.f, 0.f);
                        local_positions(t, 9) = 0;

                        //  std::cout << std::endl;

                        // for(int i = 0; i < 10; i++)
                        // {
                        //     print(positions(i));
                        // }

                        // std::cout << std::endl << "end!" << std::endl << std::endl;
                    }

                    rotations(t, 0) = PI / 4.f * sin(deg_to_rad(0.63f * t));
                    rotations(t, 1) = 1.0f * sin(deg_to_rad(2.6f * t));
                    rotations(t, 2) = 0.9f * sin(deg_to_rad(1.5f * t + 45));
                    rotations(t, 3) = 0.5f * cos(deg_to_rad(0.9f * t));

                    // rotations(t, 0) = sin(deg_to_rad(5.f * t));
                    // rotations(t, 1) = sin(deg_to_rad(1.5f * t));
                    // rotations(t, 2) = (-1.f + 2.f / 5000.f * t);
                    // rotations(t, 3) = 0.f;

                    std::cout << t << std::endl;
                    t++;
                    key = cv::waitKey(10);
                }

                FILE* fq = fopen("../resources/quaternions.bin", "wb");
                assert(fq != NULL);
                array2d_write(local_quaternions, fq);
                fclose(fq);

                FILE* fp = fopen("../resources/positions.bin", "wb");
                assert(fp != NULL);
                array2d_write(local_positions, fp);
                fclose(fp);

                FILE* fr = fopen("../resources/rotations.bin", "wb");
                assert(fr != NULL);
                array2d_write(rotations, fr);
                fclose(fr);
            }
            break;
        case 1:
            {
                int fov = 100;
                int width = 640;
                int height = 480;
                float focus = 0.0021f;
                float distance = 0.06f;
                vec3 position(0.30f, 0.16f, 0.f);
                mat3 orientation(vec3(0.f, 0.f, -1.f), vec3(0.f, 1.f, 0.f), vec3(1.f, 0.f, 0.f));

                cv::VideoCapture capture(0);
                capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
                capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                if (!capture.isOpened())
                {
                    std::cout << "error!" << std::endl;
                    return 0;
                }

                cv::Rect left_rect(0, 0, 640, 480);
                cv::Rect right_rect(640, 0, 640, 480);

                Camera camera(fov, width, height, focus, distance, position, orientation);

                Net net;
                net.Write("../resources/net.bin", 10);
                net.Read("../resources/net.bin");

                array1d<quat> local_quaternions(10);

                int t = 0;
                int key = 0;

                min_H = 150;
                max_H = 180;

                min_S = 50;
                max_V = 200;

                cv::Mat img;
                capture >> img;

                cv::imshow("img", img);
                cv::createTrackbar("H_min", "img", &min_H, 255);
                cv::createTrackbar("H_max", "img", &max_H, 255);
                cv::createTrackbar("S_min", "img", &min_S, 255);
                cv::createTrackbar("S_max", "img", &max_S, 255);
                cv::createTrackbar("V_min", "img", &min_V, 255);
                cv::createTrackbar("V_max", "img", &max_V, 255);

                while(key != 27)
                {
                    capture >> img;
                    // cv::imshow("img", img);
                    // cv::Mat jbf = Joint_Bilateral_filter(img, 4, 4, 1.0, 50, 50);
                    // cv::imshow("JBF", jbf);
                    // HSV_cut(jbf, 0, 255, 0, 255, 0, 46);
                    // cv::imshow("HSV", jbf);

                    cv::imshow("img", img);

                    cv::Mat l_img;
                    img(left_rect).copyTo(l_img);
                    // cv::imshow("left", l_img);

                    cv::Mat r_img;
                    img(right_rect).copyTo(r_img);
                    // cv::imshow("right", r_img);

                    std::vector<vec2> l_contour_points = Canny(l_img, "Canny_left", min_H, max_H, min_S, max_S, min_V, max_V);
                    std::vector<vec2> r_contour_points = Canny(r_img, "Canny_right", min_H, max_H, min_S, max_S, min_V, max_V);

                    if(l_contour_points.size() > 0 && r_contour_points.size() > 0)
                    {
                        std::vector<vec2> l_center_points = Identify_Centerline(l_img, net, l_contour_points, "SOM_left");
                        std::vector<vec2> r_center_points = Identify_Centerline(r_img, net, r_contour_points, "SOM_right");

                        std::cout << "begin!" << std::endl << std::endl;

                        array1d<vec3> positions(10);
                        array1d<quat> global_quaternions(10);

                        for(int i = 0; i < 10; i++)
                        {
                            positions(i) = camera.Rebuild(l_center_points[i], r_center_points[i]);
                        }

                        for(int i = 1; i < 10; i++)
                        {
                            vec3 v0 = normalize(vec3(0.f, 1.f, 0.f));
                            vec3 v1 = normalize(positions(i) - positions(i - 1));
                            vec3 axis = normalize(cross(v0, v1));
                            float phi = rad_to_deg(acos(dot(v0, v1)));
                            global_quaternions(i - 1) = quat(phi, axis);
                        }

                        local_quaternions(0) = global_quaternions(0);
                        for(int i = 1; i < 9; i++)
                        {
                            local_quaternions(i) = inv_quat(global_quaternions(i - 1)) * global_quaternions(i);
                        }
                        local_quaternions(9) = quat(1.f, 0.f, 0.f, 0.f);

                        std::cout << std::endl;

                        for(int i = 0; i < 10; i++)
                        {
                            print(local_quaternions(i));
                        }

                        std::cout << std::endl << "end!" << std::endl << std::endl;
                    }

                    std::cout << t << std::endl;
                    t++;
                    key = cv::waitKey(10);

                    if(key == 27)
                    {
                        // cv::imwrite("../resources/l_img.png", l_img);
                        // cv::imwrite("../resources/r_img.png", r_img);
                        l_img.copyTo(img(left_rect));
                        r_img.copyTo(img(right_rect));
                        cv::imwrite("../resources/img.png", img);
                        FILE* fp = fopen("../resources/test.bin", "wb");
                        assert(fp != NULL);
                        array1d_write(local_quaternions, fp);
                        fclose(fp);
                    }
                }
            }
            break;
        default:
            {
                int sigma = 50;
                cv::Mat img = cv::imread("../resources/img/zleft1024.PNG");
                cv::imshow("img", img);
                cv::Mat Gaussian = Gaussian_filter(img, 4, 4, 1.0, 1.5);
                // cv::imshow("GF", Gaussian);
                cv::Mat Joint_Bilateral = Joint_Bilateral_filter(img, 4, 4, 1.0, 1.5, sigma);
                // cv::imshow("JBF", Joint_Bilateral);
                // cv::createTrackbar("sigma_2", "JBF", &sigma, 1000);

                int key = 0;

                Net net;
                net.Write("../resources/net.bin", 18);
                net.Read("../resources/net.bin");

                min_H = 35;
                max_H = 77;

                // cv::waitKey(3000);

                // while (key != 27)
                {
                    // Joint_Bilateral = Joint_Bilateral_filter(img, 4, 4, 1.0, 50, sigma);
                    // cv::imshow("JBF", Joint_Bilateral);

                    std::vector<vec2> contour_points = Canny(img, "Canny", min_H, max_H, min_S, max_S, min_V, max_V);
                    std::vector<vec2> center_points = Identify_Centerline(img, net, contour_points, "SOM");

                    key = cv::waitKey(100000);
                }
            }
            break;
    } 

    std::cout << "Press any key to quit" << '\n';
    cv::waitKey(20000);
    cv::destroyAllWindows();

    return 0;
}
