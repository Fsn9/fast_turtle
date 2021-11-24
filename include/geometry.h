#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <tuple>
#include <vector>
#include <string>

double distance_between_points(double, double, double, double);

class Point2d{
	private:
		Eigen::Vector2f data;
		Eigen::Rotation2D<float> rotation;
	public:
		Point2d(double x, double y);
		double get_x();
		void set_x(double x);
		double get_y();
		void set_y(double y);
		std::string tostring();
		void rotate(double degrees);
}; 

class Circle;

class Line{
	protected:
		double denominator, numerator, slope, intercept, x1, y1, x2, y2;
		bool horizontal, vertical;
		double ordered_points_x_[2], ordered_points_y_[2];
	public:
		Line(double x1,double y1,double x2, double y2);
		std::string tostring();
		double get_slope();
		double get_intercept();
		void set_x1(double x1);
		void set_x2(double x2);
		void set_y1(double y1);
		void set_y2(double y2);
		double get_x1();
		double get_x2();
		double get_y1();
		double get_y2();
		double* get_ordered_points_x();
		double* get_ordered_points_y();
		void set_points(double, double, double, double);
		bool intersects(double x,double y);
		bool is_vertical();
		bool is_horizontal();
		std::tuple<double, double> get_midpoint();
		double get_length();
		virtual std::tuple<bool, double, double> intersects_line(Line other);
		virtual std::tuple<bool, double, double, double, double> intersects_circle(Circle circle);
};

class LineSegment : public Line{
	public:
		LineSegment(double x1, double y1, double x2, double y2);
		std::tuple<bool, double, double, double, double> intersects_circle(Circle circle) override final;
		std::tuple<bool, double, double> intersects_line(Line other) override final;
};

class Square{
	protected:
		double length, xc, yc, angle;
		std::vector<Point2d> corners;
		std::vector<LineSegment> edges;	
	public:
		std::vector<LineSegment> get_edges();
		Square(double length, double xc, double yc, double angle);
		std::string tostring();
		double get_xc();
		double get_yc();
		double get_angle();
		double get_length();
};

class Circle{
	protected:
		double radius, diameter, radius_sqr, xc, yc;	
	public:
		Circle(double xc, double yc, double radius);
		std::string tostring();
		double get_xc();
		double get_yc();
		void set_xc(double x); 
		void set_yc(double y);
		double get_radius();
		double get_diameter();
		double equation(double x, double y);
		bool intersects(double x, double y);
		bool inside(double x, double y);
		bool outside(double x, double y);
		bool intersects_circle(Circle other);
		std::tuple<bool, double, double, double, double> intersects_line(Line line);
};
#endif // GEOMETRY_H