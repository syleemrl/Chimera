#ifndef __SIM_TRANSFORM_H__
#define __SIM_TRANSFORM_H__
#include "dart/dart.hpp"
#include <queue>

namespace SIM
{
struct Pose2d
{
	Pose2d(Eigen::Vector2d _p) : pos(_p) {}
	Pose2d(Eigen::Vector2d _p, Eigen::Vector2d _d) : dir(_d), pos(_p) {}
	Eigen::Vector2d dir;
	Eigen::Vector2d pos;
};
class Transform
{
public:
	static Pose2d to2d(Eigen::Isometry3d _world, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis);
	static Eigen::Isometry3d to3d(Pose2d _local, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis);

	static Eigen::Vector2d to2dPos(Eigen::Vector3d _pos);
	static Eigen::Vector3d to3dPos(Eigen::Vector2d _pos);

	static Pose2d getLocalTransform(Pose2d _base, Pose2d _target);
	static Eigen::Isometry3d getGlobalTransform(Pose2d _from, Pose2d _to, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis);

	static Pose2d applyTransform(Pose2d _from, Pose2d _transform);
	
	static Eigen::Vector3d getYrotation(Eigen::Vector3d _rot, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis);
	static Eigen::Vector3d getYrotation(Eigen::Vector3d _dir0, Eigen::Vector3d _dir1);

};
}
#endif
