#include "camera.hpp"
#include "image_process.hpp"
#include "mat.hpp"
#include "quat.hpp"

int main(int argc, char** argv)
{
	int online;

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
				float distance = 2.f;
				vec3 position(6.f, 2.f, 0.f);
				mat3 orientation(vec3(0.f, 0.f, -1.f), vec3(0.f, 1.f, 0.f), vec3(1.f, 0.f, 0.f));

				Camera camera(fov, width, height, focus, distance, position, orientation);

				array2d<float> rotations(2500, 4);
				array2d<quat> positions(2500, 7);

				Net net;
				net.Write("../resources/net.bin", 10);
				net.Read("../resources/net.bin");

				int t = 100;
				int key = 0;

				while(t < 2500 && key != 27)
				{
					char left_name[60];
					char right_name[60];
					sprintf(left_name, "../resources/img/zleft%d.png", t);
					sprintf(right_name, "../resources/img/zright%d.png", t);

					cv::Mat l_img;
					l_img = cv::imread(left_name);

					cv::Mat r_img;
					r_img = cv::imread(right_name);

					std::vector<vec2> l_contour_points = Canny(l_img, "Canny_left");
					std::vector<vec2> r_contour_points = Canny(r_img, "Canny_right");

					if(l_contour_points.size() > 0 && r_contour_points.size() > 0)
					{
						std::vector<vec2> l_center_points = Identify_Centerline(l_img, net, l_contour_points, "SOM_left");
						std::vector<vec2> r_center_points = Identify_Centerline(r_img, net, r_contour_points, "SOM_right");

						std::cout << "begin!" << std::endl << std::endl;

						array1d<vec3> translations(7);
						array1d<quat> quaternions(7);

						for(int i = 0; i < 7; i++)
						{
							vec3 point = camera.Rebuild(l_center_points[i + 2], r_center_points[i + 2]);
							translations(i) = point;
							// print(point);
						}

						for(int i = 6; i > 0; i--)
						{
							translations(i) = normalize(translations(i) - translations(i - 1));
							// print(translations(i));
						}
						translations(0) = normalize(translations(0));

						for(int i = 0; i < 7; i++)
						{
							vec3 axis = normalize(cross(vec3(0.f, 1.f, 0.f), translations(i)));
    						float phi = rad_to_deg(acos(dot(vec3(0.f, 1.f, 0.f), translations(i))));
							// std::cout << phi << std::endl;
    						quaternions(i) = quat(phi, axis);
							// print(quaternions(i));
						}
						for(int i = 6; i > 0; i--)
						{
							quat R = inv_quat(quaternions(i - 1)) * quaternions(i);
							// print(R);
							positions(t, i) = R;
						}
						positions(t, 0) = quaternions(0);
						// positions(t, 0) = angles(0).y;
						// positions(t, 7) = angles(0).x;

						std::cout << std::endl;

						for(int i = 0; i < 8; i++)
						{
							// std::cout << positions(t, i) << std::endl;
						}

						std::cout << std::endl << "end!" << std::endl << std::endl;
					}

					rotations(t, 0) = 0.6f * cos(deg_to_rad(1.2f * t));
					rotations(t, 1) = 0.5f * cos(deg_to_rad(2.1f * t));
					rotations(t, 2) = 1.2f * sin(deg_to_rad(2.3f * t));
					rotations(t, 3) = PI * cos(deg_to_rad(0.35f * t));

					std::cout << t << std::endl;
					t++;
					key = cv::waitKey(10);
				}

				// FILE* fp = fopen("../resources/positions.bin", "wb");
				// assert(fp != NULL);
				// array2d_write(positions, fp);
				// fclose(fp);

				// FILE* fr = fopen("../resources/rotations.bin", "wb");
				// assert(fr != NULL);
				// array2d_write(rotations, fr);
				// fclose(fr);
			}
			break;
		case 1:
			{
				int fov = 100;
				int width = 640;
				int height = 480;
				float focus = 0.0021f;
				float distance = 0.06f;
				vec3 position(0.05f, 0.16f, 0.f);
				mat3 orientation(vec3(0.f, 0.f, -1.f), vec3(0.f, 1.f, 0.f), vec3(1.f, 0.f, 0.f));

				cv::VideoCapture capture(1);
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

				array1d<quat> positions(7);

				int t = 0;
				int key = 0;

				while(key != 27)
				{
					cv::Mat img;
					capture >> img;
					// cv::imshow("img", img);
					// cv::Mat jbf = Joint_Bilateral_filter(img, 4, 4, 1.0, 50, 50);
					// cv::imshow("JBF", jbf);
					// HSV_cut(jbf, 0, 255, 0, 255, 0, 46);
					// cv::imshow("HSV", jbf);

					cv::Mat l_img;
					img(left_rect).copyTo(l_img);
					// cv::imshow("left", l_img);

					cv::Mat r_img;
					img(right_rect).copyTo(r_img);
					// cv::imshow("right", r_img);

					std::vector<vec2> l_contour_points = Canny(l_img, "Canny_left");
					std::vector<vec2> r_contour_points = Canny(r_img, "Canny_right");

					if(l_contour_points.size() > 0 && r_contour_points.size() > 0)
					{
						std::vector<vec2> l_center_points = Identify_Centerline(l_img, net, l_contour_points, "SOM_left");
						std::vector<vec2> r_center_points = Identify_Centerline(r_img, net, r_contour_points, "SOM_right");

						std::cout << "begin!" << std::endl << std::endl;

						array1d<vec3> translations(7);
						array1d<quat> quaternions(7);

						for(int i = 0; i < 7; i++)
						{
							vec3 point = camera.Rebuild(l_center_points[i + 2], r_center_points[i + 2]);
							translations(i) = point;
							// print(point);
						}

						for(int i = 6; i > 0; i--)
						{
							translations(i) = normalize(translations(i) - translations(i - 1));
							// print(translations(i));
						}
						translations(0) = normalize(translations(0));

						for(int i = 0; i < 7; i++)
						{
							vec3 axis = normalize(cross(vec3(0.f, 1.f, 0.f), translations(i)));
    						float phi = rad_to_deg(acos(dot(vec3(0.f, 1.f, 0.f), translations(i))));
							// std::cout << phi << std::endl;
    						quaternions(i) = quat(phi, axis);
							// print(quaternions(i));
						}
						for(int i = 6; i > 0; i--)
						{
							quat R = inv_quat(quaternions(i - 1)) * quaternions(i);
							// print(R);
							positions(i) = R;
						}
						positions(0) = quaternions(0);

						std::cout << std::endl;

						for(int i = 0; i < 7; i++)
						{
							print(positions(i));
						}

						std::cout << std::endl << "end!" << std::endl << std::endl;
					}

					std::cout << t << std::endl;
					t++;
					key = cv::waitKey(10);

					if(key == 27)
					{
						cv::imwrite("../resources/l_img3.png", l_img);
						cv::imwrite("../resources/r_img3.png", r_img);
						FILE* fp = fopen("../resources/test.bin", "wb");
						assert(fp != NULL);
						array1d_write(positions, fp);
						fclose(fp);
					}
				}
			}
			break;
		default:
			break;
	} 

	int sigma = 50;
	cv::Mat img = cv::imread("../resources/img/6.PNG");
	cv::imshow("img", img);
	cv::Mat Gaussian = Gaussian_filter(img, 4, 4, 1.0, 1.5);
	// cv::imshow("GF", Gaussian);
	cv::Mat Joint_Bilateral = Joint_Bilateral_filter(img, 4, 4, 1.0, 1.5, sigma);
	// cv::imshow("JBF", Joint_Bilateral);
	// cv::createTrackbar("sigma_2", "JBF", &sigma, 1000);

	int key = 0;

	Net net;
	net.Write("../resources/net.bin", 7);
	net.Read("../resources/net.bin");

	// cv::waitKey(3000);

	// while (key != 27)
	{
		// Joint_Bilateral = Joint_Bilateral_filter(img, 4, 4, 1.0, 50, sigma);
		// cv::imshow("JBF", Joint_Bilateral);

		std::vector<vec2> contour_points = Canny(img, "Canny");
		std::vector<vec2> center_points = Identify_Centerline(img, net, contour_points, "SOM");

		key = cv::waitKey(100000);
	}

	// std::cout << "Press any key to quit" << '\n';
	// cv::waitKey(20000);
	// cv::destroyAllWindows();

	// quat a(10, vec3(0.f, 1.f, 0.f));
	// quat b(20, vec3(0.f, 1.f, 0.f));
	// quat c(30, vec3(1.f, 0.f, 0.f));

	// print(a);
	// print(inv_quat(a) * b);
	// print(inv_quat(c * a) * c * b);

	return 0;
}
