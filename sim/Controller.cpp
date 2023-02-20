#include "Controller.h"
#include "SkeletonBuilder.h"
#include "Functions.h"
#include "Transform.h"
#include <boost/filesystem.hpp>
#include <Eigen/QR>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <ctime>
namespace SIM
{	

Controller::
Controller(ReferenceManager* _ref, SimConfig _simConfig, int _id, bool _render)
	:mControlHz(30),mSimulationHz(120)
{
	mRender = _render;
	mReferenceManager = _ref;
		
	mEnableCollision = _simConfig.to.enableSelfCollision;
	mTimewarpingType = _simConfig.to.timewarpingType;
	if(mTimewarpingType == 2)
		mFixCycleEnd = _simConfig.to.fixCycleEnd;
	else
		mFixCycleEnd = true;
	mUseReferenceRoot = _simConfig.to.useReferenceRoot;

	mId = _id;

	if(mRender) {
		mPhaseDeltaWeight = 1;
		mFixPhaseWeight = false;
	}
	else {
		mPhaseDeltaWeight = 0;
		mFixPhaseWeight = true;
	}

	mSimPerCon = mSimulationHz / mControlHz;
	mWorld = std::make_shared<dart::simulation::World>();

	mWorld->setGravity(Eigen::Vector3d(0, -9.81, 0));
	mWorld->setTimeStep(1.0/(double)mSimulationHz);
	mWorld->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
	dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(mWorld->getConstraintSolver())->setBoxedLcpSolver(std::make_shared<dart::constraint::PgsBoxedLcpSolver>());

	mGround = SkeletonBuilder::buildFromFile(std::string(SSC_DIR)+std::string("/data/character/ground.xml"));
	mGround->getBodyNode(0)->setFrictionCoeff(1.0);
	mWorld->addSkeleton(mGround);

	mCharacter = mReferenceManager->getSkeletonCopy();
	if(mEnableCollision) {
		mCharacter->enableSelfCollisionCheck();
	}
	mWorld->addSkeleton(mCharacter);

	mNumBodyNodes = mCharacter->getNumBodyNodes();
	for(int i = 0; i < mNumBodyNodes; i++) {
		std::string name = mCharacter->getBodyNode(i)->getName();
		if(name.find("Head") != std::string::npos) {
			mHeadJoint = name;
			break;
		}
	}

	mEEJoints = _simConfig.eeJoints;
	mPosJoints = _simConfig.posJoints;
	mAxis = mReferenceManager->getMainAxis();

	mKp.resize(mCharacter->getNumDofs());
	mKd.resize(mCharacter->getNumDofs());
	
	mTorqueLimit.resize(mNumBodyNodes);
	for(int i = 0; i < mNumBodyNodes; i++) {

		int idx = mCharacter->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		int dof = mCharacter->getBodyNode(i)->getParentJoint()->getNumDofs();
		std::string name = mCharacter->getBodyNode(i)->getName();

		double kp = _simConfig.kps[name];
		mKp.block(idx, 0, dof, 1) = Eigen::VectorXd::Constant(dof, kp);
		mKd.block(idx, 0, dof, 1) = Eigen::VectorXd::Constant(dof, 2 * sqrt(kp));

		mTorqueLimit(i) = _simConfig.torqueLimits[name];
	}
	mTC = _simConfig.tc;

	mSpatialDof = mCharacter->getNumDofs() - 6;
	mSkelDof = mCharacter->getNumDofs();

	mTemporalDof = _simConfig.numTimingFunc;

	mCurrentFrame.resize(mTemporalDof);
	mCurrentFrameOnCycle.resize(mTemporalDof);
	mCurrentPhase.resize(mTemporalDof);
	mTargetPhaseDelta.resize(mTemporalDof);
	mCycleEndTime.resize(mTemporalDof);
	
	if(mTimewarpingType == 0)
		mActions = Eigen::VectorXd::Zero(mSpatialDof);
	else if(mTimewarpingType == 1)
		mActions = Eigen::VectorXd::Zero(mSpatialDof + 1);
	else
		mActions = Eigen::VectorXd::Zero(mSpatialDof + mTemporalDof);

	int posDim = (mPosJoints.size() - 1) * 6;
	int velDim = mPosJoints.size() * 3 + 3;
	int posTransDim = mEEJoints.size() * 9;
	int nextPosDim = mEEJoints.size() * 9 + 15;
	int stateDim = posDim + posTransDim + velDim + nextPosDim + 2 + mTemporalDof;

	mStates = Eigen::VectorXd::Zero(stateDim);
	
	mSyncConstraints = mReferenceManager->getSyncConstraints();
	mTorqueMean.resize(mSkelDof);

	mTargetPosition = Eigen::VectorXd::Zero(mSkelDof);
	mTargetVelocity = Eigen::VectorXd::Zero(mSkelDof);

	mPDTargetPosition = Eigen::VectorXd::Zero(mSkelDof);
	mPDTargetVelocity = Eigen::VectorXd::Zero(mSkelDof);

	mNextTargetPosition = Eigen::VectorXd::Zero(mSkelDof);
	mNumState = mStates.size();
	mNumAction = mActions.size();

	mRewardLabels.clear();
	mRewardLabels.push_back("tracking");
	mRewardLabels.push_back("task");
	mRewardLabels.push_back("time");
	mRewardLabels.push_back("pos");
	mRewardLabels.push_back("ee_pos");
	mRewardLabels.push_back("root");

	mRewardParts.resize(mRewardLabels.size(), 0.0);

}
void 
Controller::
stepTime()
{
	if(mIsTerminal)
		return;
	if(mTimewarpingType == 0) {
		mTargetPhaseDelta = mReferenceManager->getTimeStep(mCurrentFrame);
		mPhaseDelta = mTargetPhaseDelta;
	} else if(mTimewarpingType == 1) {
		mActions[mSpatialDof] = dart::math::clip(mActions[mSpatialDof], -2.0, 1.0);
		mActions[mSpatialDof] = exp(mActions[mSpatialDof]);
		
		mTargetPhaseDelta = mReferenceManager->getTimeStep(mCurrentFrame);

		mActionsTime = mActions[mSpatialDof] * mTargetPhaseDelta;

		mPhaseDelta = mPhaseDeltaWeight * mActionsTime + (1 - mPhaseDeltaWeight) * mTargetPhaseDelta;
	} else if(mTimewarpingType == 2) {
		for(int i = mSpatialDof; i < mNumAction; i++) {
			mActions[i] = dart::math::clip(mActions[i]*0.5, -0.5, 0.3);
			mActions[i] = exp(mActions[i]);
		}
		mActionsTime = mActions.tail(mTemporalDof);
		mTargetPhaseDelta = mReferenceManager->getTimeStep(mCurrentFrame);
		for(int i = 0; i < mTemporalDof; i++) {
			mActionsTime[i] *= mTargetPhaseDelta[i];
		}

		mPhaseDelta = mPhaseDeltaWeight * mActionsTime + (1 - mPhaseDeltaWeight) * mTargetPhaseDelta;
	}

	if(mFixCycleEnd) {
		mCycleEnd = true;

		for(int i = 0 ; i < mTemporalDof; i++) {

			if(mCurrentFrameOnCycle[i] < mReferenceManager->getMotionLength(i)) {
				mCurrentFrame(i) += mPhaseDelta(i);
				mCurrentFrameOnCycle(i) += mPhaseDelta(i);
				mCurrentPhase(i) += mPhaseDelta(i) / mReferenceManager->getMotionLength(i);
				if(mCurrentFrameOnCycle[i] < mReferenceManager->getMotionLength(i)) {
					mCycleEnd = false;
					mCycleEndTime(i) = mTimeElapsedEpisode;
				}
			} 
		}

		if(mCycleEnd) {
			for(int i = 0; i < mTemporalDof; i++) {
				if(mCurrentFrameOnCycle[i] >= mReferenceManager->getMotionLength(i)){
					mCurrentFrameOnCycle[i] -= mReferenceManager->getMotionLength(i);
					mCurrentPhase[i] = std::max(mCurrentPhase[i] - 1, 0.0);
				}
			}
		}
	} else {
		for(int i = 0 ; i < mTemporalDof; i++) {
			mCurrentFrame(i) += mPhaseDelta(i);
			mCurrentFrameOnCycle(i) += mPhaseDelta(i);
			mCurrentPhase(i) += mPhaseDelta(i) / mReferenceManager->getMotionLength(i);
			if(mCurrentFrameOnCycle(i) >= mReferenceManager->getMotionLength(i)){
				mCurrentFrameOnCycle(i) -= mReferenceManager->getMotionLength(i);
				mCurrentPhase(i) = std::max(mCurrentPhase[i] - 1, 0.0);
			}
		}
	}

}
void 
Controller::
setTargetPosition(Eigen::VectorXd _displacement, Eigen::VectorXd _displacementNext) 
{
	std::pair<Eigen::VectorXd, Eigen::VectorXd> result;
	if(mPrevFrameOnCycle == mCurrentFrameOnCycle) {
		result = mReferenceManager->getPositionWithDisplacement(mCurrentFrameOnCycle, _displacement, mId);
	} else {
		result = mReferenceManager->getPositionWithDisplacement(mCurrentFrameOnCycle, mPrevFrameOnCycle, mPrevRoot, _displacement, mId);
	}
	
	mTargetPosition = result.second;
	mPrevRoot = result.first.segment<6>(0);

	result = mReferenceManager->getPositionWithDisplacement(mCurrentFrameOnCycle+mTargetPhaseDelta, mCurrentFrameOnCycle, mPrevRoot, _displacementNext, mId);
	mNextTargetPosition = result.second;

	mRootMovDir = mNextTargetPosition.segment<3>(3) - mTargetPosition.segment<3>(3);
	mTargetVelocity = mCharacter->getPositionDifferences(mNextTargetPosition, mTargetPosition) / 0.0333333;
}
void 
Controller::
step()
{		

	if(mIsTerminal)
		return;
	for(int i = 0; i < mSpatialDof; i++){
		mActions[i] = dart::math::clip(mActions[i]*0.2, -0.7*M_PI, 0.7*M_PI);
	}
	mPDTargetPosition = mTargetPosition;
	for(int i = 1; i < mNumBodyNodes; i++){
		int idx = mCharacter->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		int dof = mCharacter->getBodyNode(i)->getParentJoint()->getNumDofs();
		mPDTargetPosition.block(idx, 0, dof, 1) += mActions.block((i-1)*3, 0, dof, 1);
	}

	mTorqueMean.setZero();
	for(int i = 0; i < mSimPerCon; i++){
		Eigen::VectorXd torque = mCharacter->getSPDForces(mPDTargetPosition, mKp, mKd, mWorld->getConstraintSolver());

		mCharacter->setForces(torque);
		mWorld->step(false);
		mTorqueMean += torque.cwiseAbs();

	}

	mTorqueMean /= mSimPerCon;
	if(mTorqueNorm == 0) {
		mTorqueNorm = mTorqueMean.norm();
	} else {
		mTorqueNorm = 0.1 * mTorqueMean.norm() + 0.9 * mTorqueNorm; 

	}
	mTimeElapsed += 1;
	mTimeElapsedEpisode += 1;

	if(!mUseReferenceRoot)
		alignTargetMotion(mCharacter->getPositions(), mTargetPosition);
	updateReward();
	updateRecord();
	updateTerminalInfo();
	updateState();

	mPrevPosition = mCharacter->getPositions();
	mPrevFrameOnCycle = mCurrentFrameOnCycle;

}
void
Controller::
alignTargetMotion(Eigen::VectorXd _current, Eigen::VectorXd& _target)
{
	_target(3) = _current(3);
	_target(5) = _current(5);

}
void
Controller::
updateRecord() 
{
	if(mRender) {
		mRecordTargetPosition.push_back(mTargetPosition);
		mRecordPosition.push_back(mCharacter->getPositions());
		mRecordPhase.push_back(mCurrentPhase);
	}
	mRecordParam.push_back(mCurrentFrameOnCycle);

}
void 
Controller::
clearRecord() 
{
	if(mRender) {
		mRecordPosition.clear();
		mRecordTargetPosition.clear();
		mRecordPhase.clear();
	}	
	mRecordParam.clear();
}
std::map<std::string, double> 
Controller::
getTrackingReward(Eigen::VectorXd _posCur, Eigen::VectorXd _posTarget,
				  Eigen::VectorXd _velCur, Eigen::VectorXd _velTarget)
{
	Eigen::VectorXd posSave = mCharacter->getPositions();

	Eigen::VectorXd posDiffFull = mCharacter->getPositionDifferences(_posCur, _posTarget);
	Eigen::VectorXd velDiffFull = mCharacter->getVelocityDifferences(_velCur, _velTarget);

	Eigen::VectorXd posDiff(mPosJoints.size() * 3 + 3);
	posDiff.segment<6>(0) = posDiffFull.segment<6>(0);
	// i = 0 -> root joint
	for(int i = 1; i < mPosJoints.size(); i++){
		int idx = mCharacter->getBodyNode(mPosJoints[i])->getParentJoint()->getIndexInSkeleton(0);
		posDiff.segment<3>(3*i+3) = posDiffFull.segment<3>(idx);
	}

	Eigen::VectorXd eeDiff(mEEJoints.size()*3);
	eeDiff.setZero();

	mCharacter->setPositions(_posCur);
	Eigen::Vector3d comDiff = mCharacter->getCOM();

	std::vector<Eigen::Isometry3d> eeTransforms;
	for(int i = 0; i < mEEJoints.size(); i++){
		eeTransforms.push_back(mCharacter->getBodyNode(mEEJoints[i])->getWorldTransform());
	}

	mCharacter->setPositions(_posTarget);
	comDiff -= mCharacter->getCOM();
	for(int i = 0; i < mEEJoints.size(); i++){
		Eigen::Isometry3d diff = eeTransforms[i].inverse() * mCharacter->getBodyNode(mEEJoints[i])->getWorldTransform();
		eeDiff.segment<3>(3*i) = diff.translation();
	}

	double sigPos = 0.3; 
	double sigEE = 0.3;	
	double sigCOM = 0.3;	

	double posReward = expOfSquared(posDiff, sigPos);	
	double eeReward = expOfSquared(eeDiff, sigEE);
	double comReward = expOfSquared(comDiff, sigCOM);
	std::map<std::string, double> rewards;
	rewards.clear();

	rewards.insert(std::pair<std::string, double>("pos", posReward));
	rewards.insert(std::pair<std::string, double>("ee_pos", eeReward));
	rewards.insert(std::pair<std::string, double>("com", comReward));
	mCharacter->setPositions(posSave);
	return rewards;
}
double
Controller::
getCycleEndReward()
{
	double reward = 0;
	double curCycle = 0;
	if(!mFixCycleEnd) {
		for(int i = 0; i < mTemporalDof; i++) {
			curCycle += mCurrentFrame(i) / mReferenceManager->getMotionLength(i);
		}
		curCycle /= mTemporalDof;
		if(curCycle > mNumCycle) {
			mNumCycle += 1;

			if(mTimewarpingType == 2) {
				Eigen::VectorXd curSinusoidal = mReferenceManager->toSinusoidalParam(mCurrentFrameOnCycle);
				Eigen::VectorXd prevSinusoidal = mReferenceManager->toSinusoidalParam(mPrevCycleEnd);

				Eigen::VectorXd diff = curSinusoidal - prevSinusoidal;
				reward = expOfSquared(diff, 0.8);
			}	
			mPrevCycleEnd = mCurrentFrameOnCycle;
		}

	} else {
		if(mCycleEnd) {
			mNumCycle += 1;

			if(mTimewarpingType == 2) {
				double dist = 0;
				for(int i = 0; i < mTemporalDof; i++) {
					for(int j = 0; j < mTemporalDof; j++) {
						dist += pow((mCycleEndTime(i) - mCycleEndTime(j)) / (double) mTimeElapsedEpisode, 2);
					}
				}
				dist /= (mTemporalDof * (mTemporalDof-1));
				reward = exp(-dist * 300);
			}
		}
	}
	if(mFixPhaseWeight)
		reward = 0;

	return reward;
}
double 
Controller::
getRootReward()
{
	Eigen::Vector3d dirRef = mRootMovDir;
	dirRef(1) = 0;
	dirRef.normalize();
	// dirRef << 0, 0, 1;
	Eigen::Vector3d dirCur = (mCharacter->getPositions().segment<3>(3) -  mPrevPosition.segment<3>(3));
	dirCur(1) = 0;
	double dist = dirCur.dot(dirRef);
	Eigen::Vector3d dirDiff = dirCur - dist * dirRef;
	if(dist < 0)
		dirDiff += dist * dirRef;
	double rewardDir = expOfSquared(dirDiff, 0.01);

	
	int count = 0;
	double cot = 0;
	Eigen::VectorXd velocity = mCharacter->getVelocities();
	for(int i = 0; i < mNumBodyNodes; i++) {
		if(mCharacter->getBodyNode(i)->getName().find("Tail") != std::string::npos)
			continue;
		int idx = mCharacter->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		cot += mTorqueMean.segment<3>(idx).dot(velocity.segment<3>(idx).cwiseAbs());
		count += 1;
	}
	if(dist > 0)
		cot /= (dirCur.norm() * mCharacter->getMass());
	else
		cot /= (1e-8 * mCharacter->getMass());

	cot *= 0.01;
	cot /= count;
	double penaltyCOT = std::min(3.0, pow(cot, 2)*0.5);

	return rewardDir - penaltyCOT;
}
double 
Controller::
getSyncReward()
{
	double reward = 0;
	if(mSyncConstraints.size() == 0)
		return reward;

	double phase = 0;
	for(int i = 0; i < mTemporalDof; i++) {
		phase += mCurrentPhase(i);
	}
	phase /= mTemporalDof;
	if(abs(mSyncConstraints[mSyncCount].first - phase) < 0.02) {
		
		Eigen::VectorXd curSinusoidal = mReferenceManager->toSinusoidalParam(mCurrentFrameOnCycle);
		Eigen::VectorXd targetSinusoidal = mReferenceManager->toSinusoidalParam(mSyncConstraints[mSyncCount].second);

		Eigen::VectorXd diff = curSinusoidal - targetSinusoidal;
		reward = expOfSquared(diff, 0.6);

		mSyncCount += 1;
		mSyncCount %= mSyncConstraints.size();
	}
	return reward;
}
void
Controller::
updateReward()
{
	std::map<std::string, double> rewardTracking = getTrackingReward(mCharacter->getPositions(), mTargetPosition,
																	 mCharacter->getVelocities(), mTargetVelocity);
	double rewardRoot = 1;
	if(!mUseReferenceRoot)
		rewardRoot = getRootReward();
	double rewardTorque = std::min(1.0, 0.0000001 * pow(mTorqueNorm, 2));

	double rewardTime = 0;
	if(mTimewarpingType != 0)
		rewardTime = expOfSquared(mActionsTime - mTargetPhaseDelta, 0.3);

	double rewardTotal = (0.2 * rewardRoot 
						  + 0.8 * (rewardTracking["pos"]  
								* rewardTracking["ee_pos"]))
						- 0.05 * rewardTorque;
	double rewardEnd = getCycleEndReward();
	double rewardSync = getSyncReward();

 	mRewardParts.clear();	
	if(dart::math::isNan(rewardTotal)){
		mRewardParts.resize(mRewardLabels.size(), 0.0);
	} else {
		// mRewardParts[0] ~ mRewardParts[2]: do not change order 
		mRewardParts.push_back(rewardTotal);
		mRewardParts.push_back(5 * rewardEnd + 5 * rewardSync);
		mRewardParts.push_back(rewardTime);
	
		mRewardParts.push_back(rewardTracking["pos"]);
		mRewardParts.push_back(rewardTracking["ee_pos"]);
		mRewardParts.push_back(rewardRoot);
	}
}
void
Controller::
updateTerminalInfo()
{	
	if(mIsTerminal)
		return;

	Eigen::VectorXd posSave = mCharacter->getPositions();

	Eigen::Isometry3d curRootInv = mCharacter->getRootBodyNode()->getWorldTransform().inverse();
	double rootY = mCharacter->getBodyNode(0)->getTransform().translation()[1];
	Eigen::Vector3d rootPosDiff = mCharacter->getRootBodyNode()->getWorldTransform().translation();
	Eigen::Vector3d headPosDiff = mCharacter->getBodyNode(mHeadJoint)->getWorldTransform().translation();

	mCharacter->setPositions(mTargetPosition);

	Eigen::Isometry3d rootDiff = curRootInv * mCharacter->getRootBodyNode()->getWorldTransform();
	Eigen::AngleAxisd rootDiffAngle(rootDiff.linear());
	double angle = std::fmod(rootDiffAngle.angle()+M_PI, 2*M_PI)-M_PI;
	rootPosDiff -= mCharacter->getRootBodyNode()->getWorldTransform().translation();
	headPosDiff -= mCharacter->getBodyNode(mHeadJoint)->getWorldTransform().translation();

	mCharacter->setPositions(posSave);

	// // check nan
	if(dart::math::isNan(mCharacter->getPositions()) || dart::math::isNan(mCharacter->getVelocities())){
		mIsNan = true;
		mIsTerminal = true;
		mTerminalReason = 1;
	} 
	//characterConfigration
	else if(!mRender && (rootPosDiff.norm() > mTC.rootDiffPos || headPosDiff.norm() > mTC.headDiffPos)){
		mIsTerminal = true;
		mTerminalReason = 2;
	} else if(!mRender && (rootY < mTC.rootHeightLower || rootY > mTC.rootHeightUpper)){
		mIsTerminal = true;
		mTerminalReason = 3;
	} else if(!mRender && (std::abs(angle) > mTC.rootDiffDir)){
		mIsTerminal = true;
		mTerminalReason = 4;
	} else if(mNumCycle > mTC.terminalIteration) {
		mIsTerminal = true;
		mTerminalReason = 8;
	} 
	if(mIsTerminal) {
		mReferenceManager->saveCurrentParams(mRecordParam);
		mRecordParam.clear();
	}

}
void 
Controller::
resetTime(Eigen::VectorXd _start)
{
	if((mRender && mFixCycleEnd) || _start==Eigen::VectorXd::Zero(1)) {
		mCurrentFrame = Eigen::VectorXd::Zero(mTemporalDof);
	} else
		mCurrentFrame = _start;
	mTimeElapsed = 0;
	mTimeElapsedEpisode = 0;

	mStartFrame = mCurrentFrame;	
	mCurrentFrameOnCycle = mCurrentFrame;
	for(int i = 0; i < mTemporalDof; i++) {
		mCurrentPhase(i) = mCurrentFrameOnCycle(i) / mReferenceManager->getMotionLength(i);
	}
	mPrevFrameOnCycle = mCurrentFrameOnCycle;

	mTargetPhaseDelta = mReferenceManager->getTimeStep(mCurrentFrame);
	mPhaseDelta = mTargetPhaseDelta;

	mCycleEndTime.setZero();

	mPrevCycleEnd = mCurrentFrame;	

	mNumCycle = 1;
	if(mCurrentFrame.mean() > 0)
		mNumCycle = 2;

	double phase = 0;
	for(int i = 0; i < mTemporalDof; i++) {
		phase += mCurrentPhase(i);
	}
	phase /= mTemporalDof;

	mSyncCount = mSyncConstraints.size() - 1;
	for(int i = 0; i < mSyncConstraints.size(); i++) {
		if(mSyncConstraints[i].first > phase) {
			mSyncCount = i;
			break;
		}
	}
}
void 
Controller::
reset()
{
	mIsTerminal = false;
	mIsNan = false;
	
	mWorld->reset();
	mCharacter->clearConstraintImpulses();
	mCharacter->clearInternalForces();
	mCharacter->clearExternalForces();
	
	Eigen::VectorXd pos = mTargetPosition;
	mCharacter->setPositions(pos);

	Eigen::VectorXd vel = mTargetVelocity;
	mCharacter->setVelocities(vel);	
	
	if(mEnableCollision) {
		mCharacter->enableSelfCollisionCheck();
	} else 
		mCharacter->disableSelfCollisionCheck();

	mPrevPosition = mCharacter->getPositions();

	clearRecord();
	updateRecord();
	updateState();

}
void 
Controller::
setAction(Eigen::VectorXd _action)
{
	mActions = _action;
}
Eigen::VectorXd 
Controller::
getNextPosAndVel(Eigen::VectorXd _pos, Eigen::VectorXd _vel) {
	Eigen::VectorXd ret;
	dart::dynamics::BodyNode* root = mCharacter->getRootBodyNode();
	Eigen::Isometry3d curRootInv = root->getWorldTransform().inverse();

	Eigen::VectorXd posSave = mCharacter->getPositions();
	Eigen::VectorXd velSave = mCharacter->getVelocities();

	mCharacter->setPositions(_pos);
	mCharacter->setVelocities(_vel);

	ret.resize(mEEJoints.size()*9 +15);

	for(int i=0; i < mEEJoints.size(); i++)
	{		
		Eigen::Isometry3d transform = curRootInv * mCharacter->getBodyNode(mEEJoints[i])->getWorldTransform();
		ret.segment<9>(9*i) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
							   transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2), 
							   transform.translation();
	}

	Eigen::Isometry3d transform = curRootInv * mCharacter->getRootBodyNode()->getWorldTransform();

	Eigen::Vector3d root_angular_vel_relative = curRootInv.linear() * mCharacter->getRootBodyNode()->getAngularVelocity();
	Eigen::Vector3d root_linear_vel_relative = curRootInv.linear() * mCharacter->getRootBodyNode()->getCOMLinearVelocity();

	ret.tail<15>() << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
					  transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2),
					  transform.translation(), root_angular_vel_relative, root_linear_vel_relative;

	// restore
	mCharacter->setPositions(posSave);
	mCharacter->setVelocities(velSave);

	return ret;
}
void
Controller::
updateState()
{
	if(mIsTerminal && mTerminalReason != 8){
		mStates = Eigen::VectorXd::Zero(mNumState);
		return;
	}	

	double rootHeight = mCharacter->getRootBodyNode()->getCOM()[1];

	Eigen::VectorXd p,v;
	Eigen::VectorXd velFull = mCharacter->getVelocities();
	int posDim = (mPosJoints.size() - 1) * 6;
	p.resize(posDim);
	v.resize(mPosJoints.size() * 3 + 3);
	v.segment<6>(0) = velFull.segment<6>(0);

	// i = 0 -> root joint
	for(int i = 1; i < mPosJoints.size(); i++){
		Eigen::Isometry3d transform = mCharacter->getBodyNode(mPosJoints[i])->getRelativeTransform();
		p.segment<6>(6*(i-1)) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
								 transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2);

		int idx = mCharacter->getBodyNode(mPosJoints[i])->getParentJoint()->getIndexInSkeleton(0);
		v.segment<3>(3*i + 3) = velFull.segment<3>(idx);
	}

	dart::dynamics::BodyNode* root = mCharacter->getRootBodyNode();
	Eigen::Isometry3d curRootInv = root->getWorldTransform().inverse();
	Eigen::VectorXd pTrans;
	pTrans.resize(mEEJoints.size() * 9);

	for(int i = 0; i < mEEJoints.size(); i++)
	{
		Eigen::Isometry3d transform = curRootInv * mCharacter->getBodyNode(mEEJoints[i])->getWorldTransform();
		pTrans.segment<9>(9*i) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
							      transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2), 
							      transform.translation();

	}
	
	mNextTargetPosition.segment<3>(3) = mTargetPosition.segment<3>(3) + mRootMovDir;
	Eigen::VectorXd velocity = mCharacter->getPositionDifferences(mNextTargetPosition, mTargetPosition) / 0.0333333;
	Eigen::VectorXd pNext = getNextPosAndVel(mNextTargetPosition, velocity);


	Eigen::Vector3d yVec;
	if(mAxis.first[0] == 1) {
		yVec = Eigen::Vector3d(mAxis.second[0], 0, 0);
	} else if(mAxis.first[1] == 1) {
		yVec = Eigen::Vector3d(0, mAxis.second[1], 0);
	} else {
		yVec = Eigen::Vector3d(0, 0, mAxis.second[2]);
	}
	
	Eigen::Vector3d upvec = root->getTransform().linear()*yVec;
	double upvecAngle = atan2(std::sqrt(upvec[0]*upvec[0]+upvec[2]*upvec[2]),upvec[1]);

	mStates << p, pTrans, v, upvecAngle, rootHeight, pNext, mCurrentFrameOnCycle;


}
}