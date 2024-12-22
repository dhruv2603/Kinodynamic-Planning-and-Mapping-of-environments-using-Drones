///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"

// TODO: Copy your implementation from previous projects

struct square_coord
    {
        double bl_x;
        double bl_y;
        double br_x;
        double br_y;
        double tl_x;
        double tl_y;
        double tr_x;
        double tr_y;
    };

void rotate_point(square_coord robo_coord, double p_bl[], double p_br[], double p_tr[], double p_tl[],double theta, double x, double y)
{
    // The transformation matrix to translate this frame to (x,y) and rotate by angle angle theta is given by
    double H[2][2] = {
        {cos(theta), -sin(theta)},
        {sin(theta), cos(theta)},
        };
    // The translated points of the robot in base frame
    // p_base = H*p_translated
    std::vector<double> new_p(2);
    robo_coord.bl_x = H[0][0] * p_bl[0] + H[0][1] * p_bl[1] + x;
    robo_coord.bl_y = H[1][0] * p_bl[0] + H[1][1] * p_bl[1] + y;
    robo_coord.br_x = H[0][0] * p_br[0] + H[0][1] * p_br[1] + x;
    robo_coord.br_y = H[1][0] * p_br[0] + H[1][1] * p_br[1] + y;
    robo_coord.tr_x = H[0][0] * p_tr[0] + H[0][1] * p_tr[1] + x;
    robo_coord.tr_y = H[1][0] * p_tr[0] + H[1][1] * p_tr[1] + y;
    robo_coord.tl_x = H[0][0] * p_tl[0] + H[0][1] * p_tl[1] + x;
    robo_coord.tl_y = H[1][0] * p_tl[0] + H[1][1] * p_tl[1] + y;
}

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const auto& rect : obstacles)
    {
        // Check if the point (x, y) is within the bounds of the rectangle
        if (x >= rect.x && x <= (rect.x + rect.width) &&
            y >= (rect.y) && y <= (rect.y + rect.height))
        {
            std::cout << x << rect.x << rect.width << y << rect.y << rect.height << std::endl;
            return false; // The point is not valid
        }
    }
    return true; // The point is valid
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    return false;
}

// Approach for line segment intersection based on article from geeksforgeeks
// Given three collinear points a, b, c, this function checks if point b lies on line segment 'ac'
bool CheckPointonSegment(double ax, double ay, double bx, double by, double cx, double cy) 
{ 
    if (bx <= std::max(ax, cx) && bx >= std::min(ax, cx) && 
        by <= std::max(ay, cy) && by >= std::min(ay, cy)) 
       return true; 
  
    return false; 
} 

// This function is used find the orientation of ordered triplet (a, b, c). 
// The function returns following values 
    // 0 --> Collinear 
    // 1 --> Clockwise 
    // 2 --> Counterclockwise 
int Checkorientation(double ax, double ay, double bx, double by, double cx, double cy) 
{ 
    int val = (by - ay) * (cx - bx) - 
              (bx - ax) * (cy - by); 
    
    if (val == 0) return 0;  // for collinear case
  
    return (val > 0)? 1: 2; // for clock wise or counterclock wise case
} 

// This function checks if line segments are intersecting
bool CheckSegmentsIntersect(double a1x, double a1y, double b1x, double b1y, double a2x, double a2y, double b2x, double b2y){

    int o1 = Checkorientation(a1x,a1y, b1x, b1y, a2x, a2y); 
    int o2 = Checkorientation(a1x,a1y, b1x, b1y, b2x, b2y); 
    int o3 = Checkorientation(a2x,a2y, b2x, b2y, a1x, a1y); 
    int o4 = Checkorientation(a2x,a2y, b2x, b2y, b1x, b1y); 
  
    // General case is when 
    // (a1, b1, a2) and (a1, b1, b2) have different orientations and
    // (a2, b2, a1) and (a2, b2, b1) have different orientations.
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases //
    // a1, b1 and a2 are collinear and a2 lies on segment a1b1 
    if (o1 == 0 && CheckPointonSegment(a1x,a1y, a2x,a2y, b1x, b1y)) return true; ////////////////////// Make changes here
  
    // a1, b1 and b2 are collinear and b2 lies on segment a1b1 
    if (o2 == 0 && CheckPointonSegment(a1x,a1y, b2x, b2y, b1x, b1y)) return true; 
  
    // a2, b2 and a1 are collinear and a1 lies on segment a2b2 
    if (o3 == 0 && CheckPointonSegment(a2x,a2y, a1x,a1y, b2x, b2y)) return true; 
  
     // a2, b2 and b1 are collinear and b1 lies on segment a2b2 
    if (o4 == 0 && CheckPointonSegment(a2x,a2y, b1x, b1y, b2x, b2y)) return true; 
  
    return false;

}

// Checks if the point is inside a rectangle
bool CheckPointInside(double x, double y, Rectangle r){
    return x>=r.x && x<=r.x+r.width && y>=r.y && y<=r.y+r.height;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Rotation Matrix
    // c  -s   x       x
    // s   c   y   *   y
    // 0   0   1       1

    std::vector<double> rx1, ry1, rx2, ry2;

    double threshold = 0.1; 

    double s = sin(theta);
    double c = cos(theta);

    // Multiply points by transformation matrix
    rx1.push_back(c * (-(sideLength + threshold)/2.0) - s * (-(sideLength + threshold)/2.0) + x);
    ry1.push_back(s * (-(sideLength + threshold)/2.0) + c * (-(sideLength + threshold)/2.0) + y); 
    rx2.push_back(c * ((sideLength + threshold)/2.0) - s * (-(sideLength + threshold)/2.0) + x);
    ry2.push_back(s * ((sideLength + threshold)/2.0) + c * (-(sideLength + threshold)/2.0) + y);

    rx1.push_back(c * ((sideLength + threshold)/2.0) - s * (-(sideLength + threshold)/2.0) + x);
    ry1.push_back(s * ((sideLength + threshold)/2.0) + c * (-(sideLength + threshold)/2.0) + y);
    rx2.push_back(c * ((sideLength + threshold)/2.0) - s * ((sideLength + threshold)/2.0) + x);
    ry2.push_back(s * ((sideLength + threshold)/2.0) + c * ((sideLength + threshold)/2.0) + y);

    rx1.push_back(c * (-(sideLength + threshold)/2.0) - s * (-(sideLength + threshold)/2.0) + x);
    ry1.push_back(s * (-(sideLength + threshold)/2.0) + c * (-(sideLength + threshold)/2.0) + y);
    rx2.push_back(c * (-(sideLength + threshold)/2.0) - s * ((sideLength + threshold)/2.0) + x);
    ry2.push_back(s * (-(sideLength + threshold)/2.0) + c * ((sideLength + threshold)/2.0) + y);

    rx1.push_back(c * (-(sideLength + threshold)/2.0) - s * ((sideLength + threshold)/2.0) + x);
    ry1.push_back(s * (-(sideLength + threshold)/2.0) + c * ((sideLength + threshold)/2.0) + y);  
    rx2.push_back(c * ((sideLength + threshold)/2.0) - s * ((sideLength + threshold)/2.0) + x);
    ry2.push_back(s * ((sideLength + threshold)/2.0) + c * ((sideLength + threshold)/2.0) + y);


    for (int i = 0; i < (int)obstacles.size(); i++) {
        for (int j = 0; j < (int)rx1.size(); j++){
            // Get the 4 line segments of the obstacle and check intersections with the transformed object
            if(CheckSegmentsIntersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x, obstacles[i].y, obstacles[i].x+obstacles[i].width, obstacles[i].y)){
                return false;
            } else if(CheckSegmentsIntersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x, obstacles[i].y, obstacles[i].x, obstacles[i].y+obstacles[i].height)){
                return false;
            } else if(CheckSegmentsIntersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x+obstacles[i].width, obstacles[i].y+obstacles[i].height, obstacles[i].x+obstacles[i].width, obstacles[i].y)){
                return false;
            } else if(CheckSegmentsIntersect(rx1[j], ry1[j], rx2[j], ry2[j], obstacles[i].x+obstacles[i].width, obstacles[i].y+obstacles[i].height, obstacles[i].x, obstacles[i].y+obstacles[i].height)){
                return false;
            // Check if robot is inside obstacle
            } else if(CheckPointInside(rx1[j], ry1[j], obstacles[i]) || CheckPointInside(rx2[j], ry2[j], obstacles[i])){
                return false;
            }

        }
    }
    return true;
}
