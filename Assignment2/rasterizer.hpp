//
// Created by goksu on 4/6/19.
//

#pragma once

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include "global.hpp"
#include "Triangle.hpp"
using namespace Eigen;

//定义一个命名空间 rasterizer 光栅化器
namespace rst
{
	//强枚举类型，显式定义Color值为1，Depth值为2
	enum class Buffers
	{
		Color = 1,
		Depth = 2
	};

	//重载|运算符，其实就是返回两个 Buffers 按位或的结果
	inline Buffers operator|(Buffers a, Buffers b)
	{
		return Buffers((int)a | (int)b);
	}

	//同上，返回按位与的结果
	inline Buffers operator&(Buffers a, Buffers b)
	{
		return Buffers((int)a & (int)b);
	}

	//定义两个枚举类型，线或者三角形
	enum class Primitive
	{
		Line,
		Triangle
	};

	/*
	 * For the curious : The draw function takes two buffer id's as its arguments. These two structs
	 * make sure that if you mix up with their orders, the compiler won't compile it.
	 * Aka : Type safety
	 * */

	struct pos_buf_id
	{
		int pos_id = 0;
	};

	struct ind_buf_id
	{
		int ind_id = 0;
	};

	struct col_buf_id
	{
		int col_id = 0;
	};

	class rasterizer
	{
	public:
		rasterizer(int w, int h);
		pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
		ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
		col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);

		void set_model(const Eigen::Matrix4f& m);
		void set_view(const Eigen::Matrix4f& v);
		void set_projection(const Eigen::Matrix4f& p);

		//把point点设置位color颜色的
		void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

		//根据buff的值清空缓冲区（帧缓冲区或者深度缓冲区）
		void clear(Buffers buff);

		//根据type的值（线或者三角形）来绘图
		void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);

		//返回帧缓冲区的引用（是一个vector<Vector3f>类型）
		std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

	private:
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

		//光栅化一个三角形
		void rasterize_triangle(const Triangle& t);

		// VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER

	private:
		Eigen::Matrix4f model;
		Eigen::Matrix4f view;
		Eigen::Matrix4f projection;

		std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
		std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
		std::map<int, std::vector<Eigen::Vector3f>> col_buf;

		std::vector<Eigen::Vector3f> frame_buf;

		std::vector<float> depth_buf;
		int get_index(int x, int y);

		int width, height;

		int next_id = 0;
		int get_next_id() { return next_id++; }
	};
}
