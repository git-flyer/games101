//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>


using namespace Eigen;
class Triangle {

public:
	Vector3f v[3]; /*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
	/*Per vertex values*/
	Vector3f color[3]; //color at each vertex; 每个顶点的颜色
	Vector2f tex_coords[3]; //texture u,v  每个顶点的纹理
	Vector3f normal[3]; //normal vector for each vertex 每个顶点的法向量

	//Texture *tex;
	Triangle();

	void setVertex(int ind, Vector3f ver); /*set i-th vertex coordinates */ /*设置第i个顶点的坐标*/
	void setNormal(int ind, Vector3f n); /*set i-th vertex normal vector*/  /*设置第i个顶点的法向量*/
	void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/  /*设置第i个顶点的颜色*/
	Vector3f getColor() const { return color[0] * 255; } // Only one color per triangle.  每个三角形只有一种颜色
	void setTexCoord(int ind, float s, float t); /*set i-th vertex texture coordinate*/  /*设置第i个顶点的纹理*/
	std::array<Vector4f, 3> toVector4() const;  //将三角形三个顶点坐标转化为齐次坐标
};






#endif //RASTERIZER_TRIANGLE_H
