#include "geometry.h"

// Constants
#define TO_DEG 180.0 / M_PI
#define TO_RAD M_PI / 180.0
#define HIGHEST_SLOPE 1e3
#define DENOMINATOR_TOLERANCE 0.01
#define INTERSECTION_POINT_TOLERANCE 0.01

// Classes
//Point2d
Point2d::Point2d(float x, float y){
	this->data << x,y;
}

float Point2d::get_x(){return this->data[0];}
void Point2d::set_x(float x){this->data[0] = x;}
float Point2d::get_y(){return this->data[1];}
void Point2d::set_y(float y){this->data[1] = y;}
std::string Point2d::tostring(){return "Point (" + std::to_string(data[0]) + "," + std::to_string(data[1])+")";}
void Point2d::rotate(float degrees){
	this->rotation.angle() = degrees * TO_RAD;
	this->data = this->rotation.toRotationMatrix() * this->data;
	return;
}

// Line	
Line::Line(float x1,float y1,float x2, float y2){
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

float Line::get_slope(){return this->slope;}

float Line::get_intercept(){return this->intercept;}

void Line::set_x1(float x1){this->x1 = x1;}

void Line::set_x2(float x2){this->x2 = x2;}

void Line::set_y1(float y1){this->y1 = y1;}

void Line::set_y2(float y2){this->y2 = y2;}

float Line::get_x1(){return this->x1;}
float Line::get_y1(){return this->y1;}
float Line::get_x2(){return this->x2;}
float Line::get_y2(){return this->y2;}
		
void Line::set_points(float x1, float y1, float x2, float y2){
	this->x1 = x1;
	this->y1 = y1;
	this->x2 = x2;
	this->y2 = y2;
	this->numerator = y2-y1;
	this->denominator = x2-x1;
	this->horizontal = false;
	this->vertical = false;
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

bool Line::intersects(float x,float y){return y -  this->slope * x - this->intercept <= INTERSECTION_POINT_TOLERANCE;}

bool Line::is_vertical(){return this->vertical;}

bool Line::is_horizontal(){return this->horizontal;}

std::tuple<float, float> Line::get_midpoint(){
	return {
		((this->x1 + this->x2)/2),
		((this->y1 + this->y2)/2)
	};
}

float Line::get_length(){
	return (sqrt(pow(x2 - x1, 2) +
                pow(y2 - y1, 2) * 1.0));
}
std::tuple<bool, float, float> Line::intersects_line(Line other){
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

std::tuple<bool, float, float, float, float> Line::intersects_circle(Circle* circle){ //x1, y1, x2, y2
	float discriminant;
	if (this->vertical){
		discriminant = pow(circle->get_radius(),2) - pow(this->intercept - circle->get_xc(),2);
		if (discriminant <= 0) return {false, 0,0,0,0};
		float sqrt_d = sqrt(discriminant);
		return {true, this->intercept, circle->get_yc() + sqrt_d, this-> intercept, circle->get_yc() - sqrt_d};
	}
	else{
		float slope_sqr = pow(this->slope,2);
		discriminant = pow(circle->get_radius(), 2) * (1 + slope_sqr) - pow(circle->get_yc() - this->slope * circle->get_xc() - this->intercept,2);
		if(discriminant <= 0) return {false, 0,0,0,0};
		float den = 1 + slope_sqr;
		float sqrt_d = sqrt(discriminant);
		float a = circle->get_xc() + circle->get_yc() * this->slope - this->intercept * this->slope;
		float b = this->intercept + circle->get_xc() * this->slope + circle->get_yc() * slope_sqr;
		return {true, (a + sqrt_d) / den, (b + this->slope * sqrt_d) / den, (a - sqrt_d) / den, (b - this->slope * sqrt_d) / den};
	}
}



//Circle
Circle::Circle(float xc, float yc, float radius) : xc(xc), yc(yc), radius(radius), radius_sqr(pow(this->radius,2)) {
	this->diameter = radius * 2.0;
}

std::string Circle::tostring(){return "Circle with xc: " + std::to_string(this->xc) + ", yc: " + std::to_string(this->yc)+", radius: " + std::to_string(this->radius);}

float Circle::get_xc(){return this->xc;}

float Circle::get_yc(){return this->yc;}

float Circle::get_radius(){return this->radius;}

float Circle::get_diameter(){return this->diameter;}

float Circle::equation(float x, float y){return pow(x - this->xc, 2) + pow(y - this->yc, 2);}

bool Circle::intersects(float x, float y){return this->radius_sqr - INTERSECTION_POINT_TOLERANCE <= equation(x,y) < this->radius_sqr - INTERSECTION_POINT_TOLERANCE;}

bool Circle::inside(float x, float y){return equation(x,y) <= this->radius_sqr;}

bool Circle::outside(float x, float y){return !inside(x,y);}

bool Circle::intersects_circle(Circle* other){
	float d = sqrt(equation(other->get_xc(), other->get_yc()));
	return (this->radius + other->get_radius()) > d && (d > abs(this->radius - other->get_radius()));
}


std::tuple<bool, float, float, float, float> Circle::intersects_line(Line line){
	float discriminant;
	if (line.is_vertical()){
		discriminant = pow(this->radius,2) - pow(line.get_intercept() - this->xc, 2);
		if (discriminant <= 0) return {false, 0,0,0,0};
		float sqrt_d = sqrt(discriminant);
		return {true, line.get_intercept(), this->yc + sqrt_d, line.get_intercept(), this->yc - sqrt_d};
	}
	else{
		float slope_sqr = pow(line.get_slope(),2);
		discriminant = pow(this->radius, 2) * (1 + slope_sqr) - pow(this->yc - line.get_slope() * this->xc - line.get_intercept(), 2);
		if(discriminant <= 0) return {false,0,0,0,0};
		float den = 1 + slope_sqr;
		float sqrt_d = sqrt(discriminant);
		float a = this->xc + this->yc * line.get_slope() - line.get_intercept() * line.get_slope();
		float b = line.get_intercept() + this->xc * line.get_slope() + this->yc * slope_sqr;
		return {true, (a + sqrt_d) / den, (b + line.get_slope() * sqrt_d) / den, (a - sqrt_d) / den, (b - line.get_slope() * sqrt_d) / den};
	}
}

// Square
Square::Square(float length, float xc, float yc, float angle) : length(length), xc(xc), yc(yc), angle(angle){
	// Corners
	this->corners.push_back(Point2d(xc - length * 0.5, yc + length * 0.5));
	this->corners.push_back(Point2d(xc + length * 0.5, yc + length * 0.5));
	this->corners.push_back(Point2d(xc + length * 0.5, yc - length * 0.5));
	this->corners.push_back(Point2d(xc - length * 0.5, yc - length * 0.5));

	// Edges
	for(int idx = 0; idx < this->corners.size(); idx++){
		if (idx == this->corners.size() - 1) {
			this->edges.push_back(Line(this->corners[idx].get_x(), this->corners[idx].get_y(), this->corners[0].get_x(), this->corners[0].get_y()));
		}
		else{
			this->edges.push_back(Line(this->corners[idx].get_x(), this->corners[idx].get_y(), this->corners[idx + 1].get_x(), this->corners[idx + 1].get_y()));
		}
	}	
}

std::string Square::tostring(){
	std::string repr_ = "";
	for (Point2d p:this->corners) repr_ += p.tostring() + "\n";
	for (Line l:this->edges) repr_ += l.tostring() + "\n";
	return repr_;
}

std::vector<Line> Square::get_edges(){
	return this->edges;
}

float distance_between_points(float x1, float y1, float x2, float y2){
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

float Square::get_xc(){
	return this->xc;
}
float Square::get_yc(){
	return this->yc;
}

float Square::get_angle(){
	return this->angle;
}

float Square::get_length(){
	return this->length;
}

/*bool Circle::intersects_square(Square other){
	bool intersect = false;
	for(int j = 0; j < other.get_edges().size(); j++){
		if(std::get<0>((other.get_edges()[j].intersects_circle(this)))){
			intersect = true;
		}
	}
	return intersect;
}*/