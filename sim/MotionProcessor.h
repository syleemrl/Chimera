#ifndef __SIM_MOTION_PROCESSOR_H__
#define __SIM_MOTION_PROCESSOR_H__
#include "dart/dart.hpp"
#include "SimConfigParser.h"
#include <queue>

namespace SIM
{
class MotionProcessor
{
public:
	static void alignPosition(std::vector<Eigen::VectorXd>& _data, Eigen::VectorXd _target, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis); 
	static void alignPosition(std::vector<Eigen::VectorXd>& _data, Eigen::Isometry3d _target, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis); 
	static void generateCycles(std::vector<Eigen::VectorXd>& _clip, dart::dynamics::SkeletonPtr _skel, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis, int _numCycle=2);
	static void setProcessingOption(ProcessingOption _po) { mPO = _po;}
	static void clearProcessingOption() { mPO = ProcessingOption();}
	static void IKcmaes(Eigen::VectorXd& _pos,
				 		std::vector<int> _optimDof,
				 		std::vector<std::tuple<int, int, Eigen::Vector3d>> _constraints, 
						dart::dynamics::SkeletonPtr _skel,
						std::vector<int> _interestedBodies=std::vector<int>());
	static ProcessingOption getProcessingOption() { return mPO; }
	static ProcessingOption mPO;
};
}
#endif
