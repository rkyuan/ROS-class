#include<ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
//#include <eigen3/Eigen/src/Core/Matrix.h>
using namespace std;

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}


Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
  Eigen::Affine3d affine;
    Eigen::Vector3d Oe;

    Oe(0)= pose.position.x;
    Oe(1)= pose.position.y;
    Oe(2)= pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

 return affine;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_eigen"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    tf::StampedTransform tfTransform; //need objects of this type to hold tf's
    tf::TransformListener tfListener; //create a TransformListener to listen for tf's and assemble them
    ros::Rate sleep_timer(1.0); //a timer for desired rate, e.g. 1Hz

    // make a plane:
    Eigen::Vector3d normal_vec(1,2,3);
    cout<<"normal: "<<normal_vec.transpose()<<endl;
    normal_vec/=normal_vec.norm(); // make this vector unit length
    cout<<"unit length normal: "<<normal_vec.transpose()<<endl;
    double dist = 1.23;
    cout<<"plane distance from origin: "<<dist<<endl;
    // construct a pair of axes perpendicular to the plane normal
    //define a rotation about the z axis of 90 deg:
    // [0,1,0; 1, 0, 0; 0;0;1]
    Eigen::Matrix3d Rot_z;
    Rot_z.row(0)<<0,1,0;
    Rot_z.row(1)<<1,0,0;
    Rot_z.row(2)<<0,0,1;
    cout<<"Rot_z: "<<endl;
    cout<<Rot_z<<endl;
    Eigen::Vector3d v1,v2;
    v1 = Rot_z*normal_vec;
    cout<<"v1: "<<v1.transpose()<<endl;
    // test that v1 is orthogonal to normal:
    double dotprod = v1.dot(normal_vec);
    double dotprod2 = v1.transpose()*normal_vec;
    //v2 = v1.dot(normal_vec);
    cout<<"v1 dot normal: "<<dotprod<<endl;
    v2 = v1.cross(normal_vec);
    v2/=v2.norm();
    dotprod = v2.dot(normal_vec);
    cout<<"v2: "<<v2.transpose()<<endl;
    cout<<"v2 dot normal_vec = "<<dotprod<<endl;
    // compute a vector normal to both v1 and normal_vec:
    v1 = v2.cross(normal_vec);
    cout<<"v1= "<<v1.transpose()<<"; v1 dot v2 = "<<v1.dot(v2)<<"; v1 dot normal_vec = "<<v1.dot(normal_vec)<<endl;
    int npts= 10; // create this many planar points
    Eigen::MatrixXd points_mat(3,npts);
    Eigen::Vector3d point;
    Eigen::Vector2d rand_vec;
    //generate random points that all lie on plane defined by distance and normal_vec
    for (int ipt = 0;ipt<npts;ipt++) {
    	// MatrixXd::Random returns uniform random numbers in (-1, 1).
    	rand_vec.setRandom(2,1);  // populate 2x1 vector with random values
    	//cout<<"rand_vec: "<<rand_vec.transpose()<<endl;
    	//construct a random point ON the plane normal to normal_vec at distance "dist" from origin:
    	point = dist*normal_vec + rand_vec(0)*v1+rand_vec(1)*v2;
	//save this point in the matrix "points_mat"
	points_mat.col(ipt) = point;
    }

    //all of the above points are identically on the plane defined by normal_vec and dist

    
    cout<<"random points on plane (in columns): "<<endl;
    cout<<points_mat<<endl;
    // add random noise to these points in range [-0.1,0.1]
     Eigen::MatrixXd Noise = Eigen::MatrixXd::Random(3,npts);

      points_mat = points_mat + Noise*0.1;
    cout<<"random points on plane (in columns) w/ noise: "<<endl;
    cout<<points_mat<<endl;

    //now let's see if we can discover the plane from the data:
    // first compute the centroid of the data:
    Eigen::Vector3d centroid;
    centroid = Eigen::MatrixXd::Zero(3,1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    //add all the points together:
    npts = points_mat.cols();
    cout<<"matrix has ncols = "<<npts<<endl;
    for (int ipt =0;ipt<npts;ipt++) {
	centroid+= points_mat.col(ipt);
    }
    centroid/=npts;
    cout<<"centroid: "<<centroid.transpose()<<endl;
    // subtract this centroid from all points in points_mat:
    for (int ipt =0;ipt<npts;ipt++) {
        points_mat.col(ipt)  = points_mat.col(ipt)-centroid;
    }
    //compute the covariance matrix w/rt x,y,z:
    Eigen::Matrix3d CoVar;
    CoVar = points_mat*(points_mat.transpose());
    cout<<"covariance: "<<endl;
    cout<<CoVar<<endl;
    Eigen::EigenSolver<Eigen::Matrix3d> es3d(CoVar);
    Eigen::VectorXd evals;
    //cout<<"size of evals: "<<es3d.eigenvalues().size()<<endl;
    //cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<endl;
    cout << "The eigenvalues of CoVar are:" << endl << es3d.eigenvalues().transpose() << endl;
    cout << "The matrix of eigenvectors, V, is:" << endl;
    cout<< es3d.eigenvectors() << endl << endl;

    evals= es3d.eigenvalues().real();
    cout<<"real parts of evals: "<<evals.transpose()<<endl;

    //Eigen::Matrix3d evecs;
    //evecs = es3d.eigenvectors();

    // find the min eval and correponding evec:

    double min_lambda = evals[0];
    Eigen::Vector3cd complex_vec;
    Eigen::Vector3d est_plane_normal;
    complex_vec=es3d.eigenvectors().col(0);
    //cout<<"complex_vec: "<<endl;
    //cout<<complex_vec<<endl;
    est_plane_normal = complex_vec.real();
    //cout<<"real part: "<<est_plane_normal.transpose()<<endl;
    //est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

    double lambda_test;
    int i_normal=0;
    for (int ivec=1;ivec<3;ivec++) {
        lambda_test = evals[ivec];
    	if (lambda_test<min_lambda) {
		min_lambda =lambda_test;
                i_normal= ivec; //this index is closer to index of min eval
		est_plane_normal = es3d.eigenvectors().col(ivec).real();
        }
    }
   //MatrixXdc Evecs;
   //Evecs = es3d.eigenvectors();
    // for (int i=0;i<3;i++)
    //     est_plane_normal(i) = es3d.eigenvectors().col(i_normal).real()[i];

     //est_plane_normal = es3d.eigenvectors().col(i_normal).real();
    //est_plane_normal = es3d.eigenvectors().col(i_normal).real();
    cout<<"min eval is "<<min_lambda<<", corresponding to component "<<i_normal<<endl;
    cout<<"corresponding evec (est plane normal): "<<est_plane_normal.transpose()<<endl;
    double est_dist = est_plane_normal.dot(centroid);
    cout<<"est plane distance from origin = "<<est_dist<<endl;

    return 0;
//http://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html#a8c287af80cfd71517094b75dcad2a31b
Eigen::MatrixXd A = Eigen::MatrixXd::Random(6,6);
cout << "Here is a random 6x6 matrix, A:" << endl << A << endl << endl;
Eigen::EigenSolver<Eigen::MatrixXd> es(A);
cout << "The eigenvalues of A are:" << endl << es.eigenvalues() << endl;
cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
complex<double> lambda = es.eigenvalues()[0];
cout << "Consider the first eigenvalue, lambda = " << lambda << endl;
Eigen::VectorXcd v = es.eigenvectors().col(0);
cout << "If v is the corresponding eigenvector, then lambda * v = " << endl << lambda * v << endl;
cout << "... and A * v = " << endl << A.cast<complex<double> >() * v << endl << endl;
Eigen::MatrixXcd D = es.eigenvalues().asDiagonal();
Eigen::MatrixXcd V = es.eigenvectors();
cout << "Finally, V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;



    while (ros::ok()) {
        sleep_timer.sleep();
    }
}

