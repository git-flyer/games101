// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

//�������������εĶ���
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//����һ��x,y��һ��ָ������ָ�룬�ó�����һ��Vector3f����
//����ʹ����������ķ������ж�һ�����Ƿ�����������(ʹ�ö�ά�����������)
static bool insideTriangle(int x, int y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Vector3f P0Q(x - _v[0].x(), y - _v[0].y(), 0.0);
	Vector3f P1Q(x - _v[1].x(), y - _v[1].y(), 0.0);
	Vector3f P2Q(x - _v[2].x(), y - _v[2].y(), 0.0);

	Vector3f P0P1(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0.0);
	Vector3f P1P2(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0.0);
	Vector3f P2P0(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0.0);

	if (P0Q.cross(P0P1).z() > 0) {
		return P1Q.cross(P1P2).z() > 0 && P2Q.cross(P2P0).z() > 0;
	}
	else {
		return P1Q.cross(P1P2).z() < 0 && P2Q.cross(P2P0).z() < 0;
	}
}

//�����P(x,y)��������ABC�µ���������(c1,c2,c3)����P���������ڲ������㣺c1,c2,c3�ķ�Χ����[0,1]��
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;
		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};
		//Homogeneous division
		for (auto& vec : v) {
			vec /= vec.w();
		}
		//Viewport transformation
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}
		//����һ���������������������
		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	auto v = t.toVector4();
	float x_min, y_min, x_max, y_max;
	x_min = v[0].x(); x_max = v[0].x();
	y_min = v[0].y(); y_max = v[0].y();
	//���ҵ�bounding_box�����º���������������꣬��Ϊ��float���ͣ����Ժ�����Ҫ��
	//�������
	for (int i = 0; i < 3; ++i) {
		x_min = std::min(x_min, v[i].x());
		x_max = std::max(x_max, v[i].x());
		y_min = std::min(y_min, v[i].y());
		y_max = std::max(y_max, v[i].y());
	}
	// TODO : Find out the bounding box of current triangle.
	// iterate through the pixel and find if the current pixel is inside the triangle
	// If so, use the following code to get the interpolated z value.
	//auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
	//float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
	//float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
	//z_interpolated *= w_reciprocal;
	int _x_min, _y_min, _x_max, _y_max;
	_x_min = floor(x_min); _y_min = floor(y_min);
	_x_max = ceil(x_max); _y_max = ceil(y_max);
	for (int x = _x_min; x < _x_max; ++x) {
		for (int y = _y_min; y < _y_max; ++y) {
			//t.v��������t�������������꣬��һ�� Vector3f [3] ����ʾ
			//alpha, beta, gamma�Ǹõ������������µ��������꣬�����
			//�������ڣ���ô�����ÿ��ά��ֵ���ǷǸ���
			if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
				auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;
				// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
				int tmp = get_index(x, y);
				//��Ϊpdf���Ч��ͼ z=-2 ����������ǰ�棬�������ԭ�㣬-2 > -5
				//����Ӧ�������ֵ�����ǰ�棨��Ϊ�ڴ�����͸��ͶӰ��������n��f�ķ�ת�����Ǹ�����
				if (z_interpolated > depth_buf[tmp]) {
					depth_buf[tmp] = z_interpolated;
					Eigen::Vector3f point(x, y, 1.0);
					//set_pixel���յ���һ����ά�����ά������꣬ͨ��x,y���������frame_buf�е��±�λ��
					//Ȼ�󽫸õ�����Ϊ��Ӧ����ɫ������ɫ�Ǹ�rgbֵ�ֱ�Ϊ��������ά����
					set_pixel(point, t.getColor());
				}
			}
		}
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	//��buff���λ��1ʱ�������֡��������ȫ������Ϊ��ɫ
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	//�� buff�ε�λ��1ʱ���������Ȼ�������ÿ�����ص����ֵȫ������Ϊ�����
	//��Ϊ�������������Զ�����Ǹ���������Ը�����ҵpdf�е�������ʵ����z = -5 ƽ���������
	//��Ҫ��ø�Զ������ʵ����������������Ϊ������ɣ������Ϳ������ÿ���-z����ʱ
	//z�ľ���ֵԽ��������ԽԶ���趨��
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
	}
}

//֡����������Ȼ�������Сȫ������Ϊ w*h�� ����ҵ���� 700*700
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

//��ȡ��(x,y)��frame_buf (һάvector) �е��±꣬Ҳ���ǵڼ���Ԫ��
int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

// clang-format on