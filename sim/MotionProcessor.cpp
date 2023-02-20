#include "MotionProcessor.h"
#include "SkeletonBuilder.h"
#include "BVHParser.h"
#include "Functions.h"
#include "Transform.h"
#include "cmaes.h"
#include <Eigen/QR>
#include <numeric>
#include <algorithm>
#include <iostream>
namespace SIM
{
ProcessingOption MotionProcessor::mPO;

void
MotionProcessor::
alignPosition(std::vector<Eigen::VectorXd>& _data, Eigen::VectorXd _targetPosition, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis) 
{
	
	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_data[0].head<6>());
	Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(_targetPosition.head<6>());

	Pose2d T0Local = Transform::to2d(T0, _axis);
	Pose2d T1Local = Transform::to2d(T1, _axis);

	Eigen::Isometry3d T01 = Transform::getGlobalTransform(T0Local, T1Local, _axis);

	Eigen::Isometry3d T0new = T01 * T0;	

	for(int i = 0; i < _data.size(); i++) {
		Eigen::VectorXd pos = _data[i];
		Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0new * TCurrent;

		pos.segment<6>(0) = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
		_data[i] = pos;
	}
}
void 
MotionProcessor::
alignPosition(std::vector<Eigen::VectorXd>& _data, Eigen::Isometry3d _target, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis)
{
	Eigen::Vector6d root;
	root.segment<3>(0) = logMap(_target.linear());
	root.segment<3>(3) = _target.translation();
	alignPosition(_data, root, _axis);
}
void 
MotionProcessor::
generateCycles(std::vector<Eigen::VectorXd>& _clip, dart::dynamics::SkeletonPtr _skel, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis, int _numCycle)
{
	int totalLength = _clip.size();
	
	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_clip[0].head<6>());
	Eigen::Isometry3d T0Updated;

	for(int i = 0; i < (_numCycle-1)*totalLength; i++) {
		int phase = i % totalLength;
		Eigen::VectorXd pos;
		if(phase == 0) {
			Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(_clip.back().head<6>());
			// T1.translation()(2) += 0.4;

			Pose2d T0Local = Transform::to2d(T0, _axis);
			Pose2d T1Local = Transform::to2d(T1, _axis);

			Eigen::Isometry3d T01 = Transform::getGlobalTransform(T0Local, T1Local, _axis);

			T0Updated = T01 * T0;	
		}
		pos = _clip[phase];
		Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());

		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0Updated * TCurrent;

		pos.segment<6>(0) = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
		_clip.push_back(pos);
		if(phase == 0 && mPO.blend) {
			for(int j = mPO.blendInterval; j > 0; j--) {
				double weight = 1 - j / (double)(mPO.blendInterval+1);
				Eigen::VectorXd oldPos = _clip[totalLength - j];
				_clip[totalLength - j] = weightedSumPos(oldPos, pos, weight);
			}
		}
	}
}
void 
MotionProcessor::
IKcmaes(Eigen::VectorXd& _pos,
		std::vector<int> _optimDof,
		std::vector<std::tuple<int, int, Eigen::Vector3d>> _constraints, 
		dart::dynamics::SkeletonPtr _skel,
		std::vector<int> _interestedBodies)
{
	if(_constraints.size() == 0)
		return;

	auto fitnessFunc = [=](double const *x, int N)
	{
		Eigen::VectorXd p = _pos;

			
		for(int i = 0; i < _optimDof.size(); i++) {
			p(_optimDof[i]) = x[i];

		}

		Eigen::VectorXd posDiff = _skel->getPositionDifferences(p, _pos);
		posDiff.segment<3>(0) *= 1.3;
		_skel->setPositions(p);

		Eigen::VectorXd conDiff(_constraints.size() * 3);
		for(int i = 0; i < _constraints.size(); i++) {
			if(std::get<1>(_constraints[i]) == 0) {
				// position diff
				Eigen::Vector3d trans = _skel->getBodyNode(std::get<0>(_constraints[i]))->getWorldTransform().translation();
				conDiff.segment<3>(i*3) = 1 * (trans - std::get<2>(_constraints[i]));
			} else if(std::get<1>(_constraints[i]) == 2) {
				// height diff
				Eigen::Vector3d trans = _skel->getBodyNode(std::get<0>(_constraints[i]))->getWorldTransform().translation();
				conDiff.segment<3>(i*3) = 2 * (trans - std::get<2>(_constraints[i]));
				conDiff(0) = 0;
				conDiff(2) = 0;
			} else {
				// orientation diff
				Eigen::Matrix3d linear = _skel->getBodyNode(std::get<0>(_constraints[i]))->getWorldTransform().linear();
				conDiff.segment<3>(i*3) = logMap(linear.transpose() * expMapRot(std::get<2>(_constraints[i])));	
			}
		}

		double underFloor = 0;
		for(int i = 0; i < _interestedBodies.size(); i++) {
			Eigen::Vector3d trans = _skel->getBodyNode(_interestedBodies[i])->getWorldTransform().translation();
			underFloor += std::min(0.0, trans(1) - 0.06);
	
		}
		double sum = 1 - (0.2 * expOfSquared(posDiff, 0.2) + 0.8 * expOfSquared(conDiff, 0.5)) + pow(underFloor, 2)*100;

		return sum;
	};

	auto isFeasible = [=](double const *x)
	{
		return true;
	};
		
	CMAES<double> evo;
	double *funcVals, *const*pop, *xfinal;

	const int dim = _optimDof.size();
	double xstart[dim];
	for(int j = 0; j < _optimDof.size(); j++) {
		xstart[j] = _pos(_optimDof[j]);
	}

	double stddev[dim];
	for(int j = 0; j < dim; j++) 
		stddev[j] = 0.1;

	Parameters<double> parameters;

	parameters.init(dim, xstart, stddev);
	funcVals = evo.init(parameters);
	while(!evo.testForTermination())
	{
	    pop = evo.samplePopulation(); 

		for (int j = 0; j < evo.get(CMAES<double>::PopSize); j++)
		    while (!isFeasible(pop[j]))
		        evo.reSampleSingle(j);
		    
	    for (int j = 0; j < evo.get(CMAES<double>::Lambda); j++){
	     	 funcVals[j] = fitnessFunc(pop[j], (int) evo.get(CMAES<double>::Dimension));
	    }

	   	evo.updateDistribution(funcVals);
	}

	xfinal = evo.getNew(CMAES<double>::XBestEver);
	for(int j = 0; j < _optimDof.size(); j++) {
		_pos(_optimDof[j]) = xfinal[j];
	}

}
}
