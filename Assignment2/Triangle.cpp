//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>


Triangle::Triangle() {
	v[0] << 0, 0, 0;
	v[1] << 0, 0, 0;
	v[2] << 0, 0, 0;

	color[0] << 0.0, 0.0, 0.0;
	color[1] << 0.0, 0.0, 0.0;
	color[2] << 0.0, 0.0, 0.0;

	tex_coords[0] << 0.0, 0.0;
	tex_coords[1] << 0.0, 0.0;
	tex_coords[2] << 0.0, 0.0;
}

//�����������������
void Triangle::setVertex(int ind, Vector3f ver) {
	v[ind] = ver;
}

//�ֱ�������������ķ�����
void Triangle::setNormal(int ind, Vector3f n) {
	normal[ind] = n;
}

//���õ�ind���������ɫֵ
void Triangle::setColor(int ind, float r, float g, float b) {
	if ((r < 0.0) || (r > 255.) ||
		(g < 0.0) || (g > 255.) ||
		(b < 0.0) || (b > 255.)) {
		fprintf(stderr, "ERROR! Invalid color values");
		fflush(stderr);
		exit(-1);
	}

	//color�Ǹ�0~1֮���ֵ
	color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
	return;
}
//
void Triangle::setTexCoord(int ind, float s, float t) {
	tex_coords[ind] = Vector2f(s, t);
}

std::array<Vector4f, 3> Triangle::toVector4() const
{
	std::array<Eigen::Vector4f, 3> res;
	//��v������Ԫ�أ������������������ά���꣩������һ���������1.0���4ά����
	// д��res�в�����
	std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
	return res;
}