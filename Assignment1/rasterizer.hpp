//
// Created by goksu on 4/6/19.
//

#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace rst {
	enum class Buffers
	{
		Color = 1,
		Depth = 2
	};

	inline Buffers operator|(Buffers a, Buffers b)
	{
		return Buffers((int)a | (int)b);
	}

	inline Buffers operator&(Buffers a, Buffers b)
	{
		return Buffers((int)a & (int)b);
	}

	enum class Primitive
	{
		Line,
		Triangle
	};

	/*
	 * For the curious : The draw function takes two buffer id's as its arguments.
	 * These two structs make sure that if you mix up with their orders, the
	 * compiler won't compile it. Aka : Type safety
	 * */
	struct pos_buf_id
	{
		int pos_id = 0;
	};

	struct ind_buf_id
	{
		int ind_id = 0;
	};

	class rasterizer
	{
	public:
		rasterizer(int w, int h);
		pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
		ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);

		// set_model(const Eigen::Matrix4f& m): 将内部的模型矩阵作为参数传递给光栅化器
		void set_model(const Eigen::Matrix4f& m);

		//set_view(const Eigen::Matrix4f& v) : 将视图变换矩阵设为内部视图矩阵
		void set_view(const Eigen::Matrix4f& v);


		//set_projection(const Eigen::Matrix4f& p) : 将内部的投影矩阵设为给定矩阵p，并传递给光栅化器
		void set_projection(const Eigen::Matrix4f& p);


		//set_pixel(Vector2f point, Vector3f color): 将屏幕像素点 (x, y) 设
		// 为(r, g, b) 的颜色，并写入相应的帧缓冲区位置
		void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

		void clear(Buffers buff);

		void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

		//vector<Vector3f> frame_buf: 帧缓冲对象，用于存储需要在屏幕上绘制的颜色数据。
		std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

	private:
		void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
		void rasterize_wireframe(const Triangle& t);

	private:
		Eigen::Matrix4f model;
		Eigen::Matrix4f view;
		Eigen::Matrix4f projection;

		std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
		std::map<int, std::vector<Eigen::Vector3i>> ind_buf;

		std::vector<Eigen::Vector3f> frame_buf;
		std::vector<float> depth_buf;
		int get_index(int x, int y);

		int width, height;

		int next_id = 0;
		int get_next_id() { return next_id++; }
	};
} // namespace rst
