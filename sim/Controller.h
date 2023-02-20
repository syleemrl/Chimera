#ifndef __SIM_CONTROLLER_H__
#define __SIM_CONTROLLER_H__
#include "dart/dart.hpp"
#include <queue>
#include "ReferenceManager.h"
#include "SimConfigParser.h"
namespace SIM
{
class Controller
{
public:
	Controller(ReferenceManager* _ref, SimConfig _simConfig, 
				int _id=0, bool _render=false);
	void stepTime();
	void step();
	void setTargetPosition(Eigen::VectorXd _displacement, Eigen::VectorXd _displacementNext); 

	void resetTime(Eigen::VectorXd _start=Eigen::VectorXd::Zero(1));
	void reset();
	void setAction(Eigen::VectorXd _action);
	void setPhaseDeltaWeight(double _weight) { mPhaseDeltaWeight = _weight; }
	Eigen::VectorXd getState() {return mStates; }
	std::vector<double> getRewardParts() {return mRewardParts; }
	std::vector<std::string> getRewardLabels() {return mRewardLabels; }
	
	int getNumState() {return mNumState; }
	int getNumAction() {return mNumAction; }
	
	double getTimeElapsed(){return mTimeElapsed;}
	Eigen::VectorXd getCurrentFrame(){return mCurrentFrame;}
	Eigen::VectorXd getNextFrame(){return mCurrentFrame+mPhaseDelta;}
	Eigen::VectorXd getCurrentPhase(){return mCurrentPhase;}
	Eigen::VectorXd getCurrentFrameOnCycle(){return mCurrentFrameOnCycle;}
	Eigen::VectorXd getStartFrame(){ return mStartFrame; }

	bool isTerminalState() {return mIsTerminal; }
	bool isNan() {return mIsNan;}
	int getTerminalReason() { return mTerminalReason; }

	// renderer
	Eigen::VectorXd getPositions() { return mCharacter->getPositions(); }
	Eigen::VectorXd getTargetPositions() { return mTargetPosition; }
	Eigen::VectorXd getPosition(int _idx) { return mRecordPosition[_idx]; }
	Eigen::VectorXd getTargetPosition(int _idx) { return mRecordTargetPosition[_idx]; }
	Eigen::VectorXd getPhase(int _idx) { return mRecordPhase[_idx]; }

	void setFixWeight(bool _fix) { mFixPhaseWeight = _fix; }
	bool isCycleEnd() { return mCycleEnd; }

protected:
	Eigen::VectorXd getNextPosAndVel(Eigen::VectorXd _pos, Eigen::VectorXd _vel);
	void updateReward();
	void updateState();
	void updateTerminalInfo();

	std::map<std::string, double> getTrackingReward(Eigen::VectorXd _posCur, Eigen::VectorXd _posTarget,
													Eigen::VectorXd _velCur, Eigen::VectorXd _velTarget);

	void updateRecord();
	void clearRecord();

	double getCycleEndReward();
	double getTaskReward();
	double getRootReward();
	double getSyncReward();

	void alignTargetMotion(Eigen::VectorXd _current, Eigen::VectorXd& _target);
	
	ReferenceManager* mReferenceManager;
	dart::simulation::WorldPtr mWorld;
	dart::dynamics::SkeletonPtr mGround;
	dart::dynamics::SkeletonPtr mCharacter;

	std::map<std::string, double> mTorqueMap;
	Eigen::VectorXd mKp;
	Eigen::VectorXd mKd;

	int mId;
	bool mRender;

	Eigen::VectorXd mStartFrame;
	double mTimeElapsed;

	Eigen::VectorXd mCurrentFrame;
	Eigen::VectorXd mCurrentFrameOnCycle;
	Eigen::VectorXd mPhaseDelta;
	Eigen::VectorXd mCurrentPhase;
	double mTimeElapsedEpisode;
	
	int mControlHz;
	int mSimulationHz;
	int mSimPerCon;
	
	Eigen::VectorXd mTargetPosition;
	Eigen::VectorXd mNextTargetPosition;

	Eigen::VectorXd mTargetVelocity;
	Eigen::VectorXd mTargetPhaseDelta;

	Eigen::VectorXd mPDTargetPosition;
	Eigen::VectorXd mPDTargetVelocity;

	Eigen::VectorXd mActions;
	Eigen::VectorXd mActionsTime;
	Eigen::VectorXd mStates;

	int mSpatialDof;
	int mSkelDof;
	int mTemporalDof;

	bool mIsTerminal;
	bool mIsNan;
	int mTerminalReason;

	std::string mHeadJoint;
	std::vector<std::string> mEndEffectors;
	std::vector<std::string> mPosJoints;
	std::vector<std::string> mEEJoints;
	std::vector<std::string> mRewardLabels;
	std::vector<double> mRewardParts;

	std::vector<Eigen::VectorXd> mRecordPosition;
	std::vector<Eigen::VectorXd> mRecordTargetPosition;
	std::vector<Eigen::VectorXd> mRecordPhase;

	int mNumState;
	int mNumAction;
	int mNumBodyNodes;

	double mTorqueNorm;
		
	TerminalCondition mTC;
    
    Eigen::VectorXd mCycleEndTime;
    bool mCycleEnd;

    Eigen::VectorXd mPrevPosition;

	double mPhaseDeltaWeight;
    bool mFixPhaseWeight;

    std::vector<Eigen::VectorXd> mRecordParam;
	
	Eigen::VectorXd mPrevFrameOnCycle;

    Eigen::Vector6d mPrevRoot;

    Eigen::Vector3d mRootMovDir;
    Eigen::VectorXd mTorqueMean;
    std::pair<Eigen::Vector3i, Eigen::Vector3i> mAxis;

	bool mEnableCollision;
	bool mFixCycleEnd;
	int mTimewarpingType;
	bool mUseReferenceRoot;

	Eigen::VectorXd mTorqueLimit;

	int mNumCycle;
	Eigen::VectorXd mPrevCycleEnd;

	int mEpisodeCount;

	std::vector<std::pair<double, Eigen::VectorXd>> mSyncConstraints;
	int mSyncCount;
};
}
#endif