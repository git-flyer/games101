// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp" 

constexpr double MY_PI = 3.1415926;

//�ӽǾ��󣬾��ǰ����ƽ�Ƶ�ԭ�㣬ͬʱmodelҲ����ƽ����
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

//��ҵһ���model���󣬵�ʱ��ֻ��Χ��z����ת�����ʲôҲû�������ǵ�λ����
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	return model;
}

//͸��ͶӰ����eye_fov������Ұ���½Ƕȣ�aspect_ratio�� width:height, zNear��Ŀ��ͶӰƽ���z���꣬zFar��
//��ͶӰ���ԭz���꣨�������Ƕ�����ֵ��������xoyƽ��ľ��룩
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	// TODO: Copy-paste your implementation from the previous assignment.
	//������ҵ1ʵ�ִ��뼴��: 

	//���ڴ�������zNear��zFar��������������ת���ɸ��������������λ���������������
	double l, r, b, t, n, f;
	n = -zNear, f = -zFar;
	//���ȽǶ�ת����,tan(��/2) = t/(-n)
	t = zNear * tan(MY_PI * (eye_fov / 180.0) / 2);
	r = aspect_ratio * t;
	l = -r;
	b = -t;
	//���ȹ���������������ֱ��д͸��ͶӰ����Ҳ���ԣ�����Ҳֱ�Ӹ�������
	//����ͨ��һ��squeeze��һ������ͶӰ�����ξ���任��������������
	Eigen::Matrix4f p_t_ortho_proj = Eigen::Matrix4f::Identity();
	//������ͶӰ��һ��λ�ƾ������һ����������
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
	//�ȶ���һ����դ������700*700 ��С
	rst::rasterizer r(700, 700);
	//���λ�� (0, 0, 5)
	Eigen::Vector3f eye_pos = { 0,0,5 };

	//�������������εĶ��㣬�ֱ���z=-2 �� z=-5����ƽ����
	std::vector<Eigen::Vector3f> pos
	{
			{2, 0, -2},
			{0, 2, -2},
			{-2, 0, -2},
			{3.5, -1, -5},
			{2.5, 1.5, -5},
			{-1, 0.5, -5}
	};

	//������ţ��ܹ�6�����㣩
	std::vector<Eigen::Vector3i> ind
	{
			{0, 1, 2},
			{3, 4, 5}
	};

	//����6���������ɫ
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
		//ö�����ͣ�Color = 1�� Depth = 2
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		//MVP�任
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