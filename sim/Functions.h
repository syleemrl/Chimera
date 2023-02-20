#ifndef __SIM_FUNCTIONS_H__
#define __SIM_FUNCTIONS_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <fstream>
#include <string>
namespace p = boost::python;
namespace np = boost::python::numpy;
#include "dart/dart.hpp"

namespace SIM
{
	//string
	std::string vec2Str(Eigen::VectorXd _vec);
	std::vector<double> str2Double(std::string _str);
	Eigen::Matrix3d str2Mat3d(std::string _str);
	Eigen::Vector3d str2Vec3d(std::string _str);
	std::vector<std::string> split(const std::string &s, char delim);
	std::string getDirectoryName(std::string s);
	std::string getFileName(std::string s);
	char* getWordsUntil(std::ifstream& _is, std::string keyword1, std::string keyword2="");
	void findAttribute(std::string _path, std::ifstream& _is, std::string keyword);
	std::string getString(std::string _raw);


	//orientation and rotation

    Eigen::Matrix3d eulerX2Rot(double _x);
	Eigen::Matrix3d eulerY2Rot(double _y);
	Eigen::Matrix3d eulerZ2Rot(double _z);
	Eigen::Vector3d logMap(Eigen::Matrix3d _mat);
	Eigen::Matrix3d makeSkewSymmetric(Eigen::Vector3d _v);
	Eigen::Matrix3d expMapRot(Eigen::Vector3d _q);
	Eigen::Vector3d proj(Eigen::Vector3d _u, Eigen::Vector3d _v);	
	Eigen::Matrix3d orthonormalize(Eigen::Matrix3d _mat);
	Eigen::Vector3d quat2Pos(Eigen::Quaterniond _q);
	Eigen::Quaterniond pos2Quat(Eigen::Vector3d _v);
	Eigen::VectorXd weightedSumPos(Eigen::VectorXd _target, Eigen::VectorXd _source, double _weight, bool _blendRoot=true);
	Eigen::VectorXd weightedSumVec(Eigen::VectorXd _target, Eigen::VectorXd _source, double _weight);
	Eigen::Matrix3d weightedSumRot(Eigen::Matrix3d _m1, Eigen::Matrix3d _m2, double _weight);
	Eigen::Vector3d getPosDiff(Eigen::Vector3d _p1, Eigen::Vector3d _p0);
	Eigen::Vector3d getPosDiffXZplane(Eigen::Vector3d _p1, Eigen::Vector3d _p0);
	Eigen::Vector3d rotate(Eigen::Vector3d _q0, Eigen::Vector3d _q1);
	Eigen::Vector3d rotateVector(Eigen::Quaterniond _q, Eigen::Vector3d _v);
	Eigen::Vector3d jointPositionDifferences(Eigen::Vector3d _q1, Eigen::Vector3d _q0);
	Eigen::Vector3d projectToAxis(Eigen::Vector3d _v, Eigen::Vector3d _axis);
	Eigen::VectorXd addDisplacement(Eigen::VectorXd _p, Eigen::VectorXd _d);
	Eigen::Vector3d NearestOnGeodesicCurve(Eigen::Vector3d _axis, Eigen::Vector3d _rot);
	Eigen::VectorXd addDisplacement(Eigen::VectorXd _p, Eigen::VectorXd _d);
	Eigen::VectorXd subDisplacement(Eigen::VectorXd _p0, Eigen::VectorXd _p1);
	//boost python
	np::ndarray toNumPyArray(Eigen::VectorXd _vec);
	np::ndarray toNumPyArray(std::vector<double> _vec);
	np::ndarray toNumPyArray(Eigen::MatrixXd _mat);
	np::ndarray toNumPyArray(std::vector<Eigen::VectorXd> _mat);
	np::ndarray toNumPyArray(std::vector<std::vector<double>> _mat);
	Eigen::VectorXd toEigenVector(np::ndarray _array, int _n);
	Eigen::MatrixXd toEigenMatrix(np::ndarray _array, int _n, int _m);
	std::vector<Eigen::VectorXd> toEigenVector(np::ndarray _array, int _n, int _m);	

	//vector 
	double expOfSquared(Eigen::VectorXd _vec, double _sigma = 1.0);
	bool isSame(Eigen::VectorXd _v1, Eigen::VectorXd _v2);

	//from hit_reaction
	double boxVecIntersection(Eigen::Vector3d _min, Eigen::Vector3d _max, Eigen::Vector3d _point, Eigen::Vector3d _dir);
}
#endif