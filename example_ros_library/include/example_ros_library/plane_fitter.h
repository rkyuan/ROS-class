// plane_fitter.h header file //
// wsn; Oct, 2015
// include this file in "plane_fitter.cpp", and in any main that uses this library

#ifndef PLANE_FITTER_H_
#define PLANE_FITTER_H_

#include<ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

// define a class, including a constructor, member variables and member functions
class PlaneFitter
{
public:
    PlaneFitter(); 
    void fit_points_to_plane(Eigen::MatrixXd points_array,Eigen::Vector3d &plane_normal, double &plane_dist);
    void generate_planar_points(Eigen::Vector3d normal_vec,double dist,int npts, double noise_gain,Eigen::MatrixXd &points_array);

private:
    // put private member data here;  "private" data will only be available to member functions of this class;

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
