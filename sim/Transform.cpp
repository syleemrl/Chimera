#include "Transform.h"
#include "Functions.h"
#include <boost/filesystem.hpp>
#include <Eigen/QR>
#include <fstream>
#include <numeric>
#include <algorithm>
namespace SIM
{	
Pose2d 
Transform::
to2d(Eigen::Isometry3d _world, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis)
{
	Eigen::Matrix3d m;

	m.col(_axis.first(0)) = _axis.second(0) * _world.linear().col(0);
	m.col(_axis.first(1)) = _axis.second(1) * _world.linear().col(1);
	m.col(_axis.first(2)) = _axis.second(2) * _world.linear().col(2);

	Eigen::Matrix3d adjust = Eigen::Quaterniond::FromTwoVectors(m*Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY()).toRotationMatrix();

	adjust = adjust * m;
	
	double angle = logMap(adjust)(1);

	Eigen::Vector2d dir = Eigen::Vector2d(sin(angle), -cos(angle));
	Eigen::Vector2d pos = Eigen::Vector2d(_world.translation()(0), -_world.translation()(2));

	return Pose2d(pos, dir);
}
Eigen::Isometry3d 
Transform::
to3d(Pose2d _local, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis)
{
	double angle = atan2(_local.dir(0), -_local.dir(1));
	Eigen::Vector3d dir = Eigen::Vector3d(0, angle, 0);
	Eigen::Vector3d pos = Eigen::Vector3d(_local.pos(0), 0, -_local.pos(1));

	Eigen::Matrix3d m;
	Eigen::Matrix3d local = expMapRot(dir);
	
	m.col(0) = _axis.second(0) * local.col(_axis.first(0));
	m.col(1) = _axis.second(1) * local.col(_axis.first(1));
	m.col(2) = _axis.second(2) * local.col(_axis.first(2));


	Eigen::Isometry3d _world;
	_world.linear() = m;
	_world.translation() = pos;

	return _world;
}
Eigen::Vector2d 
Transform::
to2dPos(Eigen::Vector3d _pos)
{
	return Eigen::Vector2d(_pos(0), -_pos(2));
}
Eigen::Vector3d 
Transform::
to3dPos(Eigen::Vector2d _pos)
{
	return Eigen::Vector3d(_pos(0), 0, -_pos(1));
}
Pose2d 
Transform::
getLocalTransform(Pose2d _base, Pose2d _target)
{
	Eigen::Vector2d xAxis = _base.dir;
	Eigen::Vector2d yAxis = Eigen::Vector2d(-_base.dir(1), _base.dir(0));
	xAxis.normalize();
	yAxis.normalize();

	Eigen::Vector2d dt = _target.pos - _base.pos;
	Eigen::Vector2d pos = Eigen::Vector2d(dt.dot(xAxis), dt.dot(yAxis));
	Eigen::Vector2d dir = Eigen::Vector2d(_target.dir.dot(xAxis), _target.dir.dot(yAxis));

	return Pose2d(pos, dir);
}
Eigen::Isometry3d 
Transform::
getGlobalTransform(Pose2d _from, Pose2d _to, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis)
{
	Eigen::Isometry3d from = to3d(_from, _axis);
	Eigen::Isometry3d to = to3d(_to, _axis);

	return to * from.inverse();
}
Pose2d 
Transform::
applyTransform(Pose2d _from, Pose2d _transform)
{
	Eigen::Vector2d xAxis = _from.dir;
	Eigen::Vector2d yAxis = Eigen::Vector2d(-_from.dir(1), _from.dir(0));
	double angle = atan2(_from.dir(1), _from.dir(0)) + atan2(_transform.dir(1), _transform.dir(0));

	Eigen::Vector2d pos = _transform.pos(0) * xAxis + _transform.pos(1) * yAxis + _from.pos;
	Eigen::Vector2d dir = Eigen::Vector2d(cos(angle), sin(angle));

	return Pose2d(pos, dir);
}
Eigen::Vector3d 
Transform::
getYrotation(Eigen::Vector3d _rot, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis)
{
	Eigen::Matrix3d rot = expMapRot(_rot);
	Eigen::Matrix3d m;

	m.col(_axis.first(0)) = _axis.second(0) * rot.col(0);
	m.col(_axis.first(1)) = _axis.second(1) * rot.col(1);
	m.col(_axis.first(2)) = _axis.second(2) * rot.col(2);

	
	Eigen::Matrix3d adjust = Eigen::Quaterniond::FromTwoVectors(m*Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY()).toRotationMatrix();
	adjust = adjust * m;

	return logMap(adjust);
}
Eigen::Vector3d 
Transform::
getYrotation(Eigen::Vector3d _dir0, Eigen::Vector3d _dir1)
{
	_dir0.normalize();
	_dir1.normalize();
	Eigen::Vector3d xAxis = _dir0;
	Eigen::Vector3d yAxis = Eigen::Vector3d(_dir0(2), 0, -_dir0(0));

	double x = _dir1.dot(xAxis);
	double y = _dir1.dot(yAxis);

	double angle = atan2(y, x);

	return Eigen::Vector3d(0, angle, 0);
}
}
