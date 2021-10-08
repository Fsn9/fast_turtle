#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <tuple>
#include <vector>
#include <string>
#include "utils.h"

float distance_between_points(float, float, float, float);

#ifndef POINT2D_H
#define POINT2D_H
class Point2d{
	private:
		Eigen::Vector2f data;
		Eigen::Rotation2D<float> rotation;
	public:
		Point2d(float x, float y);
		float get_x();
		void set_x(float x);
		float get_y();
		void set_y(float y);
		std::string tostring();
		void rotate(float degrees);
}; 
#endif

class Circle;

#ifndef LINE_H
#define LINE_H
class Line{
	private:
		double denominator, numerator, slope, intercept, x1, y1, x2, y2;
		bool horizontal = false, vertical = false;
	public:
		Line(float x1,float y1,float x2, float y2);
		std::string tostring();
		float get_slope();
		float get_intercept();
		void set_x1(float x1);
		void set_x2(float x2);
		void set_y1(float y1);
		void set_y2(float y2);
		float get_x1();
		float get_x2();
		float get_y1();
		float get_y2();
		void set_points(float, float, float, float);
		bool intersects(float x,float y);
		bool is_vertical();
		bool is_horizontal();
		std::tuple<float, float> get_midpoint(); //returns in <x,y> form
		float get_length();
		std::tuple<bool, float, float> intersects_line(Line other);
		//bool intersects_circle_try(Circle* circle);
		std::tuple<bool, float, float, float, float> intersects_circle(Circle* circle);
};
#endif

#ifndef SQUARE_H
#define SQUARE_H
class Square{
	protected:
		float length, xc, yc, angle;
		std::vector<Point2d> corners;
		std::vector<Line> edges;	
	public:
		std::vector<Line> get_edges();
		Square(float length, float xc, float yc, float angle);
		std::string tostring();
		float get_xc();
        float get_yc();
		float get_angle();
		float get_length();
};
#endif

#ifndef CIRCLE_H
#define CIRCLE_H
class Circle{
	protected:
		float radius, diameter, radius_sqr, xc, yc;	
	public:
		Circle(float xc, float yc, float radius);
		std::string tostring();
        float get_xc();
        float get_yc();
		float get_radius();
		float get_diameter();
		float equation(float x, float y);
		bool intersects(float x, float y);
		bool inside(float x, float y);
		bool outside(float x, float y);
		bool intersects_circle(Circle* other);
		bool intersects_square(Square other);
		std::tuple<bool, float, float, float, float> intersects_line(Line line);
};
#endif
