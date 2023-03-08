#include "Camera.hpp"
#include "Image_Process.hpp"
#include "Math.hpp"

int main(int argc, char** argv)
{
	int fov = 75;
	int width;
	int height;
	float focus = 0.1f;
	float distance = 20.f;
	vec3 position(0.f, 50.f, 0.f);
	mat3 orientation(vec3(1.f, 0.f, 0.f), vec3(0.f, 1.f, 0.f), vec3(0.f, 0.f, 1.f));

	cv::Mat l_img = cv::imread("4.png");
	std::vector<vec2<float>> l_contour_points = Canny(l_img);
	std::vector<vec2<float>> l_center_points = Identify_Centerline(l_img, l_contour_points);

	width = l_img.cols;
	height = l_img.rows;

	Camera camera(fov, width, height, focus, distance, position, orientation);

	for(auto center_point : l_center_points)
	{
		vec3<float> point = camera.Rebuild(center_point, center_point);
		std::cout << point << std::endl;
	}

	std::cout << "Press any key to quit" << '\n';
	cv::waitKey(20000);
	cv::destroyAllWindows();

	return 0;
}
