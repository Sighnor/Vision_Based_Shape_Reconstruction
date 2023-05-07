#include "camera.hpp"
#include "image_process.hpp"
#include "mat.hpp"

int main(int argc, char** argv)
{
	int fov = 75;
	int width = 960;
	int height = 540;
	float focus = 0.1f;
	float distance = 20.f;
	vec3 position(0.f, 50.f, 0.f);
	mat3 orientation(vec3(1.f, 0.f, 0.f), vec3(0.f, 1.f, 0.f), vec3(0.f, 0.f, 1.f));

	cv::Mat l_img = cv::imread("../resourses/6.png");
	cv::imshow("img", l_img);
	std::vector<vec2> l_contour_points = Canny(l_img);
	std::vector<vec2> l_center_points = Identify_Centerline(l_img, l_contour_points);

	cv::Mat r_img = cv::imread("../resourses/6.png");
	cv::imshow("img", r_img);
	std::vector<vec2> r_contour_points = Canny(r_img);
	std::vector<vec2> r_center_points = Identify_Centerline(r_img, r_contour_points);

	Camera camera(fov, width, height, focus, distance, position, orientation);

	for(int i = 0; i < 7; i++)
	{
		vec3 point = camera.Rebuild(l_center_points[i], r_center_points[i]);
		print(point);
	}

	std::cout << "Press any key to quit" << '\n';
	cv::waitKey(20000);
	cv::destroyAllWindows();

	return 0;
}
