// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp" 

constexpr double MY_PI = 3.1415926;

//视角矩阵，就是把相机平移到原点，同时model也跟着平移了
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, 1, -eye_pos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

//作业一里的model矩阵，当时就只是围绕z轴旋转，这个什么也没做，就是单位矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	return model;
}

//透视投影矩阵，eye_fov代表视野上下角度，aspect_ratio是 width:height, zNear是目的投影平面的z坐标，zFar是
//待投影点的原z坐标（这里它们都是正值，代表离xoy平面的距离）
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	// TODO: Copy-paste your implementation from the previous assignment.
	//复制作业1实现代码即可: 

	//由于传进来的zNear和zFar是正数，所以先转化成负数，这样三角形画出来就是正的了
	double l, r, b, t, n, f;
	n = -zNear, f = -zFar;
	//首先角度转弧度,tan(θ/2) = t/(-n)
	t = zNear * tan(MY_PI * (eye_fov / 180.0) / 2);
	r = aspect_ratio * t;
	l = -r;
	b = -t;
	//首先构造两个正交矩阵，直接写透视投影矩阵也可以，虎书也直接给出答案了
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

	if (argc == 2)
	{
		command_line = true;
		filename = std::string(argv[1]);
	}
	//先定义一个光栅化器，700*700 大小
	rst::rasterizer r(700, 700);
	//相机位置 (0, 0, 5)
	Eigen::Vector3f eye_pos = { 0,0,5 };

	//定义两个三角形的顶点，分别在z=-2 和 z=-5两个平面上
	std::vector<Eigen::Vector3f> pos
	{
			{2, 0, -2},
			{0, 2, -2},
			{-2, 0, -2},
			{3.5, -1, -5},
			{2.5, 1.5, -5},
			{-1, 0.5, -5}
	};

	//定义序号（总共6个顶点）
	std::vector<Eigen::Vector3i> ind
	{
			{0, 1, 2},
			{3, 4, 5}
	};

	//定义6个顶点的颜色
	std::vector<Eigen::Vector3f> cols
	{
			{217.0, 238.0, 185.0},
			{217.0, 238.0, 185.0},
			{217.0, 238.0, 185.0},
			{185.0, 217.0, 238.0},
			{185.0, 217.0, 238.0},
			{185.0, 217.0, 238.0}
	};

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);
	auto col_id = r.load_colors(cols);

	int key = 0;
	int frame_count = 0;

	if (command_line)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27)
	{
		//枚举类型，Color = 1， Depth = 2
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		//MVP变换
		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';
	}

	return 0;
}
// clang-format on