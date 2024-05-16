#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
		-eye_pos[2], 0, 0, 0, 1;

	view = translate * view;

	return view;
}


/*
	逐个元素地构建模型变换矩
	阵并返回该矩阵。在此函数中，你只需要实现三维中绕z轴旋转的变换矩阵，
	而不用处理平移与缩放
*/
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	//很明显 rotation_angle 是旋转角度
	//只需要返回绕z轴旋转该旋转角度的矩阵即可
	//需要注意的一点是rotation_angle需要转化为弧度
	rotation_angle = rotation_angle * MY_PI / 180;
	//下面这一行代码的意义：在定义该矩阵变量时，创建一个同尺寸同数据类型的单位阵，
	// 对其初始化，这里它是4*4的
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	model << cos(rotation_angle), -1 * sin(rotation_angle), 0, 0,
		sin(rotation_angle), cos(rotation_angle), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return model;
}

/*
	使用给定的参数逐个元素地构建透视投影矩阵并返回
	该矩阵
	//eyefov表示视野角度，aspect_ratio表示xy的比例 (即width:height)
	zNear,zFar 是课里讲的 n 和 f， 都是负值
*/
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	//由于传进来的zNear和zFar是正数，所以先转化成负数，这样三角形画出来就是正的了
	double l, r, b, t, n, f;
	n = -zNear, f = -zFar;
	//首先角度转弧度,tan(θ/2) = t/(-n)
	t = zNear * tan(MY_PI * (eye_fov / 180.0) / 2);
	r = aspect_ratio * t;
	l = -r;
	b = -t;
	//首先构造两个单位矩阵，直接写透视投影矩阵也可以，虎书也直接给出答案了
	//但是通过一次squeeze和一次正交投影，两次矩阵变换乘起来会更好理解
	Eigen::Matrix4f p_t_ortho_proj = Eigen::Matrix4f::Identity();
	//该正交投影是一个位移矩阵左乘一个放缩矩阵
	Eigen::Matrix4f orthographic_proj = Eigen::Matrix4f::Identity();
	p_t_ortho_proj << n, 0, 0, 0,
		0, n, 0, 0,
		0, 0, n + f, -n * f,
		0, 0, 1, 0;
	orthographic_proj << 2 / (r - l), 0, 0, (r + l) / (l - r),
		0, 2 / (t - b), 0, (t + b) / (b - t),
		0, 0, 2 / (n - f), (n + f) / (f - n),
		0, 0, 0, 1;
	return orthographic_proj * p_t_ortho_proj;
}

int main(int argc, const char** argv)
{
	float angle = 0;
	bool command_line = false;
	std::string filename = "output.png";

	if (argc >= 3) {
		command_line = true;
		angle = std::stof(argv[2]); // -r by default
		if (argc == 4) {
			filename = std::string(argv[3]);
		}
		else
			return 0;
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = { 0, 0, 5 };

	std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

	std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;
	int frame_count = 0;

	if (command_line) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		if (key == 'a') {
			angle += 10;
		}
		else if (key == 'd') {
			angle -= 10;
		}
	}

	return 0;
}
