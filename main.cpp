#include "camera.hpp"
#include "image_process.hpp"
#include "mat.hpp"

int main(int argc, char** argv)
{
	int fov = 90;
	int width = 540;
	int height = 540;
	float focus = 0.05f;
	float distance = 1.f;
	vec3 position(5.f, 2.f, 0.f);
	mat3 orientation(vec3(0.f, 0.f, -1.f), vec3(0.f, 1.f, 0.f), vec3(1.f, 0.f, 0.f));

	array2d<vec3> rotations(500, 3);
	array2d<vec3> positions(500, 7);

	for(int t = 0; t < 500; t++)
	{
		char left_name[50];
        char right_name[50];
        sprintf(left_name, "../resources/zleft%d.png", t);
        sprintf(right_name, "../resources/zright%d.png", t);

		cv::Mat l_img = cv::imread(left_name);
		cv::imshow("left", l_img);
		std::vector<vec2> l_contour_points = Canny(l_img);
		std::vector<vec2> l_center_points = Identify_Centerline(l_img, l_contour_points, "SOM_left");

		cv::Mat r_img = cv::imread(right_name);
		cv::imshow("right", r_img);
		std::vector<vec2> r_contour_points = Canny(r_img);
		std::vector<vec2> r_center_points = Identify_Centerline(r_img, r_contour_points, "SOM_right");

		Camera camera(fov, width, height, focus, distance, position, orientation);

		for(int i = 0; i < 7; i++)
		{
			vec3 point = camera.Rebuild(l_center_points[i], r_center_points[i]);
			positions(t, i) = point;
			print(point);
		}

		rotations(t, 0) = 4.4 * t;
		rotations(t, 1) = 1.f * t;
		rotations(t, 2) = 0.f;

		cv::waitKey(1);
	}

	// Ray test1(vec3(1.f, 0.f, 0.f), normalize(vec3(-1.f, 0.f, -1.f)));
	// Ray test2(vec3(-1.f, 0.f, 0.f), normalize(vec3(1.f, 0.f, -1.f)));

	// print(Get_Intersection(test1, test2));

	std::cout << "Press any key to quit" << '\n';
	cv::waitKey(20000);
	cv::destroyAllWindows();

	FILE* f = fopen("../resources/positions.bin", "wb");
    assert(f != NULL);
	array2d_write(positions, f);
	fclose(f);

	f = fopen("../resources/rotations.bin", "wb");
    assert(f != NULL);
	array2d_write(rotations, f);
	fclose(f);

	return 0;
}
