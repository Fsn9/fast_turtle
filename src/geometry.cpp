#include "geometry.h"
#include "utils.h"

#define TO_DEG 180.0 / M_PI
#define TO_RAD M_PI / 180.0
#define HIGHEST_SLOPE 1e3
#define DENOMINATOR_TOLERANCE 0.01
#define INTERSECTION_POINT_TOLERANCE 0.01

//Point2d
Point2d::Point2d(double x, double y){
	this->data << x,y;
}

double Point2d::get_x(){return this->data[0];}
void Point2d::set_x(double x){this->data[0] = x;}
double Point2d::get_y(){return this->data[1];}
void Point2d::set_y(double y){this->data[1] = y;}
std::string Point2d::tostring(){return "Point (" + std::to_string(data[0]) + "," + std::to_string(data[1])+")";}
void Point2d::rotate(double degrees){
	this->rotation.angle() = degrees * TO_RAD;
	this->data = this->rotation.toRotationMatrix() * this->data;
	return;
}

// Line	
Line::Line(double x1,double y1,double x2, double y2){
	this->horizontal = false;
	this->vertical = false;
	this->set_points(x1,y1,x2,y2);
}

std::string Line::tostring(){
	return "Line (slope:" + std::to_string(this->slope)
	+ ", intercept:" 
	+ std::to_string(this->intercept) + ")"
	+ ", x1: " + std::to_string(this->x1) 
	+ ", y1: " + std::to_string(this->y1) 
	+ ", x2: " + std::to_string(this->x2) 
	+ ", y2: " + std::to_string(this->y2)
	+ "| horizontal: " + bool_to_string(this->horizontal) + ", vertical: " + bool_to_string(this->vertical)
	+"\n";
}

double Line::get_slope(){return this->slope;}

double Line::get_intercept(){return this->intercept;}

void Line::set_x1(double x1){this->x1 = x1;}

void Line::set_x2(double x2){this->x2 = x2;}

void Line::set_y1(double y1){this->y1 = y1;}

void Line::set_y2(double y2){this->y2 = y2;}

double Line::get_x1(){return this->x1;}
double Line::get_y1(){return this->y1;}
double Line::get_x2(){return this->x2;}
double Line::get_y2(){return this->y2;}

double* Line::get_ordered_points_x()
{
	return ordered_points_x_;
}

double* Line::get_ordered_points_y()
{
	return ordered_points_y_;
}
		
void Line::set_points(double x1, double y1, double x2, double y2){
	this->x1 = x1;
	this->y1 = y1;
	this->x2 = x2;
	this->y2 = y2;
	this->numerator = y2-y1;
	this->denominator = x2-x1;
	this->horizontal = false;
	this->vertical = false;
	ordered_points_x_[0] = x1;
	ordered_points_x_[1] = x2;
	ordered_points_y_[0] = y1;
	ordered_points_y_[1] = y2;
	std::sort(ordered_points_x_, ordered_points_x_ + individual_size<double*>(ordered_points_x_));
	std::sort(ordered_points_y_, ordered_points_y_ + individual_size<double*>(ordered_points_y_));
	// If line is not vertical
	if (this->denominator != 0 || abs(this->denominator) >= DENOMINATOR_TOLERANCE){
		this->slope = numerator / (double)denominator;
		this->intercept = y2 - this->slope * x2;
		if (abs(this->slope) > HIGHEST_SLOPE){
			this->slope = HIGHEST_SLOPE;
			this->intercept = x2;
		}
	}
	// If line is vertical
	else{
		this->slope = HIGHEST_SLOPE;
		this->intercept = x2;
	}
	// If line is horizontal
	if (this->slope == 0) this->horizontal = true;
	if (abs(this->slope) >= HIGHEST_SLOPE) {
		this->vertical = true;
	}
}

bool Line::intersects(double x,double y){return y -  this->slope * x - this->intercept <= INTERSECTION_POINT_TOLERANCE;}

bool Line::is_vertical(){return this->vertical;}

bool Line::is_horizontal(){return this->horizontal;}

std::tuple<double, double> Line::get_midpoint(){
	return {
		0.5 * (this->x1 + this->x2),
		0.5 * (this->y1 + this->y2)
	};
}

double Line::get_length(){
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
std::tuple<bool, double, double> Line::intersects_line(Line other){
	if (this->slope == other.get_slope()) return {false,0,0};
	else if(!this->vertical && other.vertical) return {true, other.get_intercept(), this->slope * other.get_intercept() + this->intercept};
	else if(this->vertical && !other.vertical) return {true, this->intercept, other.get_slope() * this->intercept + other.get_intercept()};
	else{
		return {
			true, 
			(other.get_intercept() - this->intercept) / (this->slope - other.get_slope()),
			(other.get_intercept() * this->slope - this->intercept * other.get_slope())/(this->slope - other.get_slope())
			};
	}

}

std::tuple<bool, double, double, double, double> Line::intersects_circle(Circle circle){ //x1, y1, x2, y2
	double discriminant;
	if (this->vertical){
		discriminant = pow(circle.get_radius(),2) - pow(this->intercept - circle.get_xc(),2);
		if (discriminant <= 0) return {false, 0,0,0,0};
		double sqrt_d = sqrt(discriminant);
		return {true, this->intercept, circle.get_yc() + sqrt_d, this-> intercept, circle.get_yc() - sqrt_d};
	}
	else{
		double slope_sqr = pow(this->slope,2);
		discriminant = pow(circle.get_radius(), 2) * (1 + slope_sqr) - pow(circle.get_yc() - this->slope * circle.get_xc() - this->intercept,2);
		if(discriminant <= 0) return {false, 0,0,0,0};
		double den = 1 + slope_sqr;
		double sqrt_d = sqrt(discriminant);
		double a = circle.get_xc() + circle.get_yc() * this->slope - this->intercept * this->slope;
		double b = this->intercept + circle.get_xc() * this->slope + circle.get_yc() * slope_sqr;
		return {true, (a + sqrt_d) / den, (b + this->slope * sqrt_d) / den, (a - sqrt_d) / den, (b - this->slope * sqrt_d) / den};
	}
}

//Line Segment
LineSegment::LineSegment(double x1, double y1, double x2, double y2) : Line(x1,x2,y1,y2){}

std::tuple<bool, double, double, double, double> LineSegment::intersects_circle(Circle circle)
{
	std::tuple<bool, double, double, double, double> points = Line::intersects_circle(circle);
	if(std::get<0>(points))
	{
		if(!in_between(ordered_points_x_[0], std::get<1>(points), ordered_points_x_[1])
		|| !in_between(ordered_points_x_[0], std::get<3>(points), ordered_points_x_[1])
		|| !in_between(ordered_points_y_[0], std::get<2>(points), ordered_points_y_[1])
		|| !in_between(ordered_points_y_[0], std::get<4>(points), ordered_points_y_[1]))
		{
			return {false, std::get<1>(points), std::get<2>(points), std::get<3>(points),std::get<4>(points)};
		}
	}
	return points;
}
std::tuple<bool, double, double> LineSegment::intersects_line(Line other)
{
	std::tuple<bool, double, double> points = Line::intersects_line(other);
	if(std::get<0>(points))
	{
		if(!in_between(ordered_points_x_[0], std::get<1>(points), ordered_points_x_[1]) 
		|| !in_between(ordered_points_y_[0], std::get<2>(points), ordered_points_y_[1]))
		{
			return {false, std::get<1>(points), std::get<2>(points)};
		}
	}
	return points;
}

//Circle
Circle::Circle(double xc, double yc, double radius) : xc(xc), yc(yc), radius(radius), radius_sqr(pow(this->radius,2)) {
	this->diameter = radius * 2.0;
}

std::string Circle::tostring(){return "Circle with xc: " + std::to_string(this->xc) + ", yc: " + std::to_string(this->yc)+", radius: " + std::to_string(this->radius);}

double Circle::get_xc(){return this->xc;}

double Circle::get_yc(){return this->yc;}

void Circle::set_xc(double x){this->xc = x;}

void Circle::set_yc(double y){this->yc = y;}

double Circle::get_radius(){return this->radius;}

double Circle::get_diameter(){return this->diameter;}

double Circle::equation(double x, double y){return pow(x - this->xc, 2) + pow(y - this->yc, 2);}

bool Circle::intersects(double x, double y){return this->radius_sqr - INTERSECTION_POINT_TOLERANCE <= equation(x,y) < this->radius_sqr - INTERSECTION_POINT_TOLERANCE;}

bool Circle::inside(double x, double y){return equation(x,y) <= this->radius_sqr;}

bool Circle::outside(double x, double y){return !inside(x,y);}

bool Circle::intersects_circle(Circle other){
	double d = sqrt(equation(other.get_xc(), other.get_yc()));
	return (this->radius + other.get_radius()) > d && (d > abs(this->radius - other.get_radius()));
}


std::tuple<bool, double, double, double, double> Circle::intersects_line(Line line){
	double discriminant;
	if (line.is_vertical()){
		discriminant = pow(this->radius,2) - pow(line.get_intercept() - this->xc, 2);
		if (discriminant <= 0) return {false, 0,0,0,0};
		double sqrt_d = sqrt(discriminant);
		return {true, line.get_intercept(), this->yc + sqrt_d, line.get_intercept(), this->yc - sqrt_d};
	}
	else{
		double slope_sqr = pow(line.get_slope(),2);
		discriminant = pow(this->radius, 2) * (1 + slope_sqr) - pow(this->yc - line.get_slope() * this->xc - line.get_intercept(), 2);
		if(discriminant <= 0) return {false,0,0,0,0};
		double den = 1 + slope_sqr;
		double sqrt_d = sqrt(discriminant);
		double a = this->xc + this->yc * line.get_slope() - line.get_intercept() * line.get_slope();
		double b = line.get_intercept() + this->xc * line.get_slope() + this->yc * slope_sqr;
		return {true, (a + sqrt_d) / den, (b + line.get_slope() * sqrt_d) / den, (a - sqrt_d) / den, (b - line.get_slope() * sqrt_d) / den};
	}
}

// Square
Square::Square(double length, double xc, double yc, double angle) : length(length), xc(xc), yc(yc), angle(angle){
	// Corners
	this->corners.push_back(Point2d(xc - length * 0.5, yc + length * 0.5));
	this->corners.push_back(Point2d(xc + length * 0.5, yc + length * 0.5));
	this->corners.push_back(Point2d(xc + length * 0.5, yc - length * 0.5));
	this->corners.push_back(Point2d(xc - length * 0.5, yc - length * 0.5));

	// Edges
	for(int idx = 0; idx < this->corners.size(); idx++){
		if (idx == this->corners.size() - 1) {
			this->edges.push_back(LineSegment(this->corners[idx].get_x(), this->corners[idx].get_y(), this->corners[0].get_x(), this->corners[0].get_y()));
		}
		else{
			this->edges.push_back(LineSegment(this->corners[idx].get_x(), this->corners[idx].get_y(), this->corners[idx + 1].get_x(), this->corners[idx + 1].get_y()));
		}
	}	
}

std::string Square::tostring(){
	std::string repr_ = "";
	for (Point2d p:this->corners) repr_ += p.tostring() + "\n";
	for (Line l:this->edges) repr_ += l.tostring() + "\n";
	return repr_;
}

std::vector<LineSegment> Square::get_edges(){
	return this->edges;
}

double distance_between_points(double x1, double y1, double x2, double y2){
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

double Square::get_xc(){
	return this->xc;
}
double Square::get_yc(){
	return this->yc;
}

double Square::get_angle(){
	return this->angle;
}

double Square::get_length(){
	return this->length;
}
