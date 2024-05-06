#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
int main() {
	//���峣��Pi
	const double Pi = acos(-1);
	Eigen::Vector3f k1(2.0, 1.0, 1.0);
	Eigen::Matrix3f ts, ts1;
	//��ת������ts����ʱ����ת45��
	ts << cos(Pi / 4), -1 * sin(Pi / 4), 0, sin(Pi / 4), cos(Pi / 4), 0, 0, 0, 1;
	//ƽ�ƾ�����ts1��x=x + 1, y = y + 2, z����
	ts1 << 1, 0, 1, 0, 1, 2, 0, 0, 1;
	std::cout << ts1 * ts * k1 << std::endl;
}