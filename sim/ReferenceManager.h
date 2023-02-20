#ifndef __SIM_REFERENCE_MANAGER_H__
#define __SIM_REFERENCE_MANAGER_H__

#include <tuple>
#include <mutex>
#include "dart/dart.hpp"
#include "SkeletonBuilder.h"
#include "SimConfigParser.h"
#include "MotionProcessor.h"
#include "ParamTree.h"

namespace SIM
{
struct JoinInfo
{
	int phaseIdx;
	int level;

	int rootBaseIdx;
	int rootIdx;

	int rootParentBaseIdx;
	int rootParentIdx;

	std::vector<int> bodyList;
};
class ReferenceManager
{
public:
	ReferenceManager(dart::dynamics::SkeletonPtr _skel, 
			 		std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis = std::pair<Eigen::Vector3i, Eigen::Vector3i>(Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(1, 1, 1)),
					int _slaves=1);
	ReferenceManager(std::vector<dart::dynamics::SkeletonPtr> _skels, int _slaves=1);
	ReferenceManager(SimConfig _simConfig, int _slaves=1);

	void loadMotionFromBVH(std::string _bvh, int _idx=0, bool _configExists=false);
	void loadContact(std::string _file, int _idx=0);
	
	int getNumBase() { return mNumBase; }
	
	std::map<std::string, double> getBaseContact(double _t, int _idx=0);
	std::map<int, double> getContact(Eigen::VectorXd _t);

	Eigen::VectorXd getBaseFrame(double _t, int _idx=0, bool _secondCycle=false, bool _global=false);
	Eigen::VectorXd getBaseFrame(double _t, double _tPrev, Eigen::VectorXd _prev, int _idx=0);
	Eigen::VectorXd getFrame(Eigen::VectorXd _t, bool _secondCycle=false, int _id=0);
	Eigen::VectorXd getFrame(Eigen::VectorXd _t, Eigen::VectorXd _tPrev, Eigen::VectorXd _prev, int _id=0);
	std::pair<Eigen::VectorXd, Eigen::VectorXd> getPositionWithDisplacement(Eigen::VectorXd _t,  Eigen::VectorXd _tprev, Eigen::VectorXd _prev, Eigen::VectorXd _displacement, int _id=0);
	std::pair<Eigen::VectorXd, Eigen::VectorXd> getPositionWithDisplacement(Eigen::VectorXd _t, Eigen::VectorXd _displacement, int _id=0);
	int getMotionLength(int _idx=0);
	int getNumTimingFunc() {return mNumTimingFunc; }

	Eigen::VectorXd getTimeStep(Eigen::VectorXd _t);
	Eigen::VectorXd getRandomStart();

	void setTimeStep();

	void setCurrentCharacter(dart::dynamics::SkeletonPtr _skel, std::map<int, BodyInfo> _map);
	dart::dynamics::SkeletonPtr getSkeletonCopy() { return mCurCharacter->cloneSkeleton(); }
	dart::dynamics::SkeletonPtr getBaseSkelCopy(int _idx) { return mSkels[_idx]->cloneSkeleton(); }
	int getBodyPhaseIdx(int _idx) { return mBodyPhaseMap[_idx]; }

	void setJoinInfo();
	JoinInfo getJoinInfo(int _idx) { return mJoinInfo[_idx]; }
	BodyInfo getBodyInfo(int _idx) { return mCurCharacterMap[_idx]; }
	int getPhaseBaseMapping(int _idx) { return mPhaseBaseMap[_idx]; }

	int getSkeletonDof() { return mCurCharacter->getNumDofs(); }
	
	void saveCurrentParams(std::vector<Eigen::VectorXd> _param);
	bool optimizeReference();
	void pretrainReference();
	std::vector<Eigen::VectorXd> buildParamTree();
	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> getTrainingData(); 
	double getDensity(Eigen::VectorXd _p) { return mCurParamTree->getDensity(_p); }

	void calculateHeightOffset();
	Eigen::VectorXd getPosSynthesized(Eigen::VectorXd _t, Eigen::VectorXd _tPrev, Eigen::VectorXd _prev);
	Eigen::VectorXd getPosSynthesized(Eigen::VectorXd _t, Eigen::VectorXd _weight, bool _contact=true);
	Eigen::VectorXd getPosSynthesized(Eigen::VectorXd _t, bool _contact=true);

	Eigen::Vector3d getCurrentOrientation(dart::dynamics::SkeletonPtr _skel, int _idx, int _rootIdx=0);
	std::vector<std::tuple<int, int, Eigen::Vector3d>> getFootConstraints(Eigen::VectorXd& _pos, Eigen::VectorXd _t, std::vector<int> _interestedBodies=std::vector<int>(), bool _smooth=false);
	Eigen::VectorXd getStyle(Eigen::VectorXd _p);
	void addConstraint(Eigen::VectorXd _phase, Eigen::VectorXd _style, double _speed,
					   std::vector<bool> _activatePhase, std::vector<bool> _activateStyle, bool _activateSpeed);
	Eigen::VectorXd getBaseStyle() { return mBaseStyle;}
	double getBaseSpeed() { return mSpeed;}
	std::vector<Eigen::VectorXd> getConstraintSync() { return mConstraintSync; }
	std::vector<std::tuple<int, double, double>> getConstraintStyle() { return mConstraintStyle; }
	std::pair<Eigen::Vector3i, Eigen::Vector3i> getMainAxis() {return mBaseAxisOrder[mPhaseBaseMap[mBodyPhaseMap[0]]]; }
	Eigen::VectorXd toSinusoidalParam(Eigen::VectorXd _param);
	std::vector<Eigen::VectorXd> getInitialTiming() { return mInitialTiming; }
	std::vector<std::pair<double, Eigen::VectorXd>> getSyncConstraints();
protected:
	std::vector<dart::dynamics::SkeletonPtr> mSkels;
	std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>> mBaseAxisOrder;

	std::vector<Eigen::VectorXd> mRecordCurParams;
	ParamTree* mCurParamTree;
	ParamTree* mOptimizedParamTree;

	int mNumBase;

	std::vector<int> mMotionLength;

	std::vector<std::vector<Eigen::VectorXd>> mBaseMotion;
	std::vector<std::vector<Eigen::VectorXd>> mBaseMotionGlobal;

	std::vector<JoinInfo> mJoinInfo;

	std::vector<std::vector<std::map<std::string, double>>> mContact;

	std::map<int, BodyInfo> mCurCharacterMap;
	std::map<int, Eigen::Vector3d> mRotMap;

	dart::dynamics::SkeletonPtr mCurCharacter;
	
	int mNumTimingFunc;
	std::map<int, int> mBodyPhaseMap;
	std::map<int, int> mPhaseBaseMap;
	//std::vector<std::vector<double>> mTimeStep;

	Eigen::VectorXd mTimeStep;
	std::vector<Eigen::VectorXd> mInitialTiming;

	std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;

	int mNumSlaves;
	std::mutex mLock;

	double mSpeed;
	Eigen::VectorXd mBaseStyle;
	std::vector<Eigen::VectorXd> mConstraintSync;
	std::vector<std::tuple<int, double, double>> mConstraintStyle;
};
}

#endif