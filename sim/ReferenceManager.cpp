#include "ReferenceManager.h"
#include "Functions.h"
#include "BVHParser.h"
#include "Transform.h"
#include "cmaes.h"
#include <numeric>
#include <deque>
namespace SIM
{
ReferenceManager::
ReferenceManager(dart::dynamics::SkeletonPtr _skel, std::pair<Eigen::Vector3i, Eigen::Vector3i> _axis, int _slaves):mRD(), mMT(mRD()), mUniform(0.0, 1.0)
{
	MotionProcessor::clearProcessingOption();

	mNumBase = 1;
	mNumTimingFunc = 1;
	mSkels.push_back(_skel);
	mBaseAxisOrder.push_back(_axis);

	mNumSlaves = _slaves;
	mSpeed = 1;

	std::map<int, BodyInfo> bodyMap;
	for(int i = 0; i < mSkels[0]->getNumBodyNodes(); i++) {
		BodyInfo bi;
		bi.skelIdx = 0;
		bi.skelBodyIdx = i;
		bi.scale = 1.0;

		bodyMap.insert(std::pair<int, BodyInfo>(i, bi));
	}

	mPhaseBaseMap.insert(std::pair<int, int>(0, 0));
	for(int i = 0 ; i < mSkels[0]->getNumBodyNodes(); i++) {
		mBodyPhaseMap.insert(std::pair<int, int>(i, 0));
	}

	mBaseAxisOrder.push_back(std::pair<Eigen::Vector3i, Eigen::Vector3i>
							 (Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(1, 1, 1)));

	setCurrentCharacter(mSkels[0], bodyMap);
}
ReferenceManager::
ReferenceManager(std::vector<dart::dynamics::SkeletonPtr> _skels, int _slaves):mRD(), mMT(mRD()), mUniform(0.0, 1.0)
{
	MotionProcessor::clearProcessingOption();

	mNumBase = _skels.size();
	mNumTimingFunc = _skels.size();
	mSkels = _skels;

	mNumSlaves = _slaves;
	mSpeed = 1;
	
	for(int i = 0; i < mNumBase; i++) {
		mPhaseBaseMap.insert(std::pair<int, int>(i, i));
		mBaseAxisOrder.push_back(std::pair<Eigen::Vector3i, Eigen::Vector3i>
								 (Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(1, 1, 1)));
	}
}
ReferenceManager::
ReferenceManager(SimConfig _simConfig, int _slaves):mRD(), mMT(mRD()), mUniform(0.0, 1.0)
{
	mNumSlaves = _slaves;

	MotionProcessor::setProcessingOption(_simConfig.po);

	mBaseAxisOrder = _simConfig.axisOrder;
	mNumBase = _simConfig.numBase;
	mNumTimingFunc = _simConfig.numTimingFunc;
	mPhaseBaseMap = _simConfig.timingFuncBase;

	std::vector<std::string> skelnames = _simConfig.baseSkels;
	for(int i = 0 ; i < skelnames.size(); i++) {
	    skelnames[i] = std::string(SSC_DIR) + std::string("/data/character/") + skelnames[i];
	    mSkels.push_back(SkeletonBuilder::buildFromFile(skelnames[i]));
	}

	std::vector<std::string> motionnames = _simConfig.baseMotions;
	for(int i = 0 ; i < motionnames.size(); i++) {
	    motionnames[i] =  std::string(SSC_DIR) + std::string("/data/motion/") + motionnames[i]; 
		loadMotionFromBVH(motionnames[i], i, true);
	}

	if(_simConfig.isAssembled) {
		std::string assembledPath = std::string(SSC_DIR) + std::string("/data/character/assembled/") + _simConfig.assembledSkel; 	
		std::pair<SIM::SkelInfo, std::map<int, Eigen::Vector3d>> result = SkeletonBuilder::assembleFromFile(assembledPath, skelnames);

		mRotMap = result.second;
		dart::dynamics::SkeletonPtr assembledSkel = result.first.skel;
		std::map<int, BodyInfo> assembledBodyMap = result.first.bodyMap;

		for(int i = 0; i < assembledSkel->getNumBodyNodes(); i++) {
			std::string name = assembledSkel->getBodyNode(i)->getName();
			mBodyPhaseMap.insert(std::pair<int, int>(i, _simConfig.timingFuncs[name]));
		}
		SkeletonBuilder::scaleBodyNode(mSkels, assembledSkel, assembledBodyMap);
		setCurrentCharacter(assembledSkel, assembledBodyMap);
	} else {
		std::map<int, BodyInfo> bodyMap;
		for(int i = 0; i < mSkels[0]->getNumBodyNodes(); i++) {
			BodyInfo bi;
			bi.skelIdx = 0;
			bi.skelBodyIdx = i;
			bi.scale = 1.0;

			bodyMap.insert(std::pair<int, BodyInfo>(i, bi));
		}

		for(int i = 0; i < mSkels[0]->getNumBodyNodes(); i++) {
			std::string name = mSkels[0]->getBodyNode(i)->getName();
			mBodyPhaseMap.insert(std::pair<int, int>(i, _simConfig.timingFuncs[name]));
		}

		setCurrentCharacter(mSkels[0], bodyMap);
	}

	if(_simConfig.c.loadConstraint) {
		mSpeed = _simConfig.c.speed;
		mBaseStyle = _simConfig.c.baseStyle;
		mConstraintSync = _simConfig.c.constraintSync;
		mConstraintStyle = _simConfig.c.constraintStyle;
	} else {
		mBaseStyle = Eigen::VectorXd::Ones(mNumTimingFunc);
		mSpeed = 1;
	}
	setTimeStep();
	calculateHeightOffset();

	Eigen::VectorXd unit = getTimeStep(Eigen::VectorXd::Zero(mNumTimingFunc));
	mOptimizedParamTree = new ParamTree(mNumTimingFunc, unit);
	mCurParamTree = new ParamTree(mNumTimingFunc, unit);


}
void 
ReferenceManager::
loadMotionFromBVH(std::string _bvh, int _idx, bool _configExists)
{
	int skelIdx = mPhaseBaseMap[_idx];
	auto pv = BVHParser::loadPosAndVelFromBVH(_bvh, mSkels[skelIdx]);
	std::vector<Eigen::VectorXd> motionBVH;
	for(int i = 0; i < pv.first.size(); i++) {
		motionBVH.push_back(pv.first[i]);
	}

	mBaseMotion.push_back(motionBVH);
	mMotionLength.push_back(motionBVH.size());
	loadContact(_bvh+std::string(".contact"), _idx);

	MotionProcessor::generateCycles(mBaseMotion[_idx], mSkels[skelIdx], mBaseAxisOrder[skelIdx], 2);
	std::vector<Eigen::VectorXd> poslist;
	for(int i = 0; i < mBaseMotion[_idx].size(); i++) {
		Eigen::VectorXd pos(mBaseMotion[_idx][i].rows());
		pos.segment<6>(0) = mBaseMotion[_idx][i].segment<6>(0);
		mSkels[skelIdx]->setPositions(mBaseMotion[_idx][i]);
		for(int j = 1; j < mSkels[skelIdx]->getNumBodyNodes(); j++) {
			int idx = mSkels[skelIdx]->getBodyNode(j)->getParentJoint()->getIndexInSkeleton(0);
			pos.segment<3>(idx) = logMap(mSkels[skelIdx]->getBodyNode(j)->getTransform().linear());
		}
		poslist.push_back(pos);
	}
	mBaseMotionGlobal.push_back(poslist);

}
void
ReferenceManager::
loadContact(std::string _file, int _idx)
{
	std::string filename = split(_file, '/').back();
	std::string file = std::string(SSC_DIR) + std::string("/data/annotation/") + filename;
	int motionLength = mMotionLength[_idx];

	std::ifstream is(file);
	char buffer[256];

	if(!is)
	{
		std::cout << "Can't open file: " << file << std::endl;
		std::vector<std::map<std::string, double>> curContact;

		for(int i = 0; i < motionLength; i++) {
			std::map<std::string, double> contactFrame;
			curContact.push_back(contactFrame);
		}
		mContact.push_back(curContact);
	} else {
		int n;
		is >> n;

		std::map<std::string, std::vector<double>> contactRaw;
		std::vector<std::string> joints;
		for(int i = 0; i < n; i++) {
			is >> buffer;
			std::vector<double> c;
			for(int j = 0; j < motionLength; j++)
				c.push_back(0.5);
			contactRaw.insert(std::pair<std::string, std::vector<double>>(buffer, c));
			joints.push_back(buffer);
		}
		while(is >> buffer)
		{
			std::string joint = buffer;
			int a, b, c;
			is >> a;
			is >> b;
			is >> c;

			for(int i = a; i <= b; i++) {
				contactRaw[joint][i] = c;
			}
		}

		std::vector<std::map<std::string, double>> curContact;

		for(int i = 0; i < motionLength; i++) {
			std::map<std::string, double> contactFrame;
			for(int j = 0; j < joints.size(); j++) {
				std::string joint = joints[j];
				contactFrame.insert(std::pair<std::string, double>(joint, contactRaw[joint][i]));
			}
			curContact.push_back(contactFrame);
		}
	
		mContact.push_back(curContact);
		is.close();
	}
}
Eigen::VectorXd 
ReferenceManager::
getBaseFrame(double _t, int _idx, bool _secondCycle, bool _global)
{
	if(_secondCycle) 
		_t += getMotionLength(_idx);

	std::vector<Eigen::VectorXd> motion;
	if(_global) 
		motion = mBaseMotionGlobal[_idx];
	else
		motion = mBaseMotion[_idx];

	if(motion.size()-1 < _t) {
	 	return motion.back();
	}
	int k0 = (int) std::floor(_t);
	int k1 = (int) std::ceil(_t);	

	if (k0 == k1) {
		return motion[k0];
	} else {
		Eigen::VectorXd p = weightedSumPos(motion[k0], motion[k1], (_t-k0));
		return p;	
	}	
}
Eigen::VectorXd  
ReferenceManager::
getBaseFrame(double _t, double _tPrev, Eigen::VectorXd _prev, int _idx)
{
	Eigen::VectorXd prevFrame;
	Eigen::VectorXd curFrame;

	if(_t >= _tPrev) {
		prevFrame = getBaseFrame(_tPrev, _idx);
		curFrame = getBaseFrame(_t, _idx);
	} else {
		prevFrame = getBaseFrame(_tPrev, _idx);
		curFrame = getBaseFrame(_t, _idx, true);
	}
	double height = curFrame(4);

	Eigen::Isometry3d prevRef = dart::dynamics::FreeJoint::convertToTransform(prevFrame.segment<6>(0));
	Eigen::Isometry3d curRef = dart::dynamics::FreeJoint::convertToTransform(curFrame.segment<6>(0));
	Eigen::Isometry3d moved = prevRef.inverse() * curRef;

	Eigen::Isometry3d prev = dart::dynamics::FreeJoint::convertToTransform(_prev.segment<6>(0));
	Eigen::Isometry3d cur = prev * moved;

	curFrame.segment<3>(0) = logMap(cur.linear());
	curFrame.segment<3>(3) = cur.translation();
	curFrame(4) = height;


	return curFrame;
}
std::map<std::string, double>
ReferenceManager::
getBaseContact(double _t, int _idx)
{
	_t = std::fmod(_t, getMotionLength(_idx));
	std::vector<std::map<std::string, double>> contact = mContact[_idx];

	if(contact.size() == 0) {
		return std::map<std::string, double>();
	}
	if(contact.size()-1 < _t) {
	 	return contact[contact.size()-1];
	}

	int k0 = (int) std::floor(_t);
	int k1 = (int) std::ceil(_t);	

	if(k0 == k1)
		return contact[k0];
	else {
		std::map<std::string, double> blended;
		for(auto it = contact[k0].begin(); it != contact[k0].end(); it++) {
			std::string key = it->first;
			if(contact[k0][key] == contact[k1][key])
				blended.insert(std::pair<std::string, double>(key, contact[k0][key]));
			else
				blended.insert(std::pair<std::string, double>(key, 0.5));

		}
		return blended;
	}
}
Eigen::VectorXd 
ReferenceManager::
getFrame(Eigen::VectorXd _t, bool _secondCycle, int _id)
{
	for(int i = 0; i < _t.rows(); i++) {
		_t(i) = std::fmod(_t(i), getMotionLength(i));
	}

	Eigen::VectorXd pos(mCurCharacter->getNumDofs());
	for(int i = 0; i < mNumTimingFunc; i++) {
		Eigen::VectorXd f;
		if(i == mBodyPhaseMap[0]) {
			f = getBaseFrame(_t(i), i, _secondCycle);
		} else {
			f = getBaseFrame(_t(i), i);
		}

		Eigen::VectorXd	posBase = f;	
		
		for(int j = 0; j < mCurCharacter->getNumBodyNodes(); j++) {
			if(i == mBodyPhaseMap[j]) {
				int idx = mCurCharacter->getBodyNode(j)->getParentJoint()->getIndexInSkeleton(0);
				int bodyIdx = mCurCharacterMap[j].skelBodyIdx;

				if(j == 0) {
					pos.segment<6>(0) = posBase.segment<6>(0);
				} else if(bodyIdx == 0) {
					pos.segment<3>(idx) = posBase.segment<3>(0);
				} else {
					pos.segment<3>(idx) = posBase.segment<3>(3 * bodyIdx + 3);
				}
				if(mRotMap.find(j) != mRotMap.end()) {
					Eigen::Vector3d rot = mRotMap[j];
					pos.segment<3>(idx) = rotate(pos.segment<3>(idx), rot);
				}

			}
		}
	}
	return pos;

}
Eigen::VectorXd 
ReferenceManager::
getFrame(Eigen::VectorXd _t, Eigen::VectorXd _tPrev, Eigen::VectorXd _prev, int _id) 
{
	Eigen::VectorXd t2 = _t;
	Eigen::VectorXd tPrev2 = _tPrev;
	for(int i = 0; i < _t.rows(); i++) {
		t2(i) = std::fmod(_t(i), getMotionLength(i));
		tPrev2(i) = std::fmod(_tPrev(i), getMotionLength(i));
	}

	int rootp = mBodyPhaseMap[0];

	Eigen::VectorXd prevFrame;
	Eigen::VectorXd curFrame;

	if(t2[rootp] >= tPrev2[rootp]) {
		prevFrame = getFrame(_tPrev, false, _id);
		curFrame = getFrame(_t, false, _id);
	} else {
		prevFrame = getFrame(_tPrev, false, _id);
		curFrame = getFrame(_t, true, _id);
	}
	double height = curFrame(4);

	Eigen::Isometry3d prevRef = dart::dynamics::FreeJoint::convertToTransform(prevFrame.segment<6>(0));
	Eigen::Isometry3d curRef = dart::dynamics::FreeJoint::convertToTransform(curFrame.segment<6>(0));
	Eigen::Isometry3d moved = prevRef.inverse() * curRef;
	
	Eigen::Isometry3d prev = dart::dynamics::FreeJoint::convertToTransform(_prev.segment<6>(0));
	Eigen::Isometry3d cur = prev * moved;

	curFrame.segment<3>(0) = logMap(cur.linear());
	curFrame.segment<3>(3) = cur.translation();
	
	std::vector<Eigen::VectorXd> p;
	curFrame.segment<3>(0) = logMap(curRef.linear());
	curFrame.segment<3>(3) = curRef.translation();

	p.push_back(curFrame);
	
	Eigen::Vector6d root;
	root.segment<3>(0) = logMap(cur.linear());
	root.segment<3>(3) = cur.translation();
	root(4) = height;

	MotionProcessor::alignPosition(p, root, mBaseAxisOrder[mCurCharacterMap[0].skelIdx]);
	curFrame = p[0];

	return curFrame;
}
std::map<int, double>
ReferenceManager::
getContact(Eigen::VectorXd _t)
{
	for(int i = 0; i < mNumTimingFunc; i++) {
		_t(i) = std::fmod(_t(i), getMotionLength(i));
	}

	std::map<int, double> contact;
	for(int i = 0; i < mCurCharacter->getNumBodyNodes(); i++) {
		BodyInfo bi = mCurCharacterMap[i];

		int skelIdx = bi.skelIdx;
		int bodyIdx = bi.skelBodyIdx;
		int phaseIdx = mBodyPhaseMap[i];
		std::string bodyname = mSkels[skelIdx]->getBodyNode(bodyIdx)->getName();
		
		double phase = std::fmod(_t[phaseIdx], mMotionLength[phaseIdx]);
		if(phase >= mMotionLength[phaseIdx] -1) 
			phase = mMotionLength[phaseIdx] - 1;
		if(mContact[phaseIdx][0].find(bodyname) != mContact[phaseIdx][0].end()) {
			int k0 = (int) std::floor(phase);
			int k1 = (int) std::ceil(phase);	

			if(k0 == k1) {
				contact.insert(std::pair<int, double>(i, mContact[phaseIdx][k0][bodyname]));	
			}
			else {
				if(mContact[phaseIdx][k0][bodyname] == mContact[phaseIdx][k1][bodyname]) {
					contact.insert(std::pair<int, double>(i, mContact[phaseIdx][k0][bodyname]));	
				}
				else
					contact.insert(std::pair<int, double>(i, 0.5));	
			}
		}
	}
	return contact;
}
void 
ReferenceManager::
setCurrentCharacter(dart::dynamics::SkeletonPtr _skel, std::map<int, BodyInfo> _map)
{
	mCurCharacterMap = _map;
	mCurCharacter = _skel;

	if(mBodyPhaseMap.size() == 0) {
		for(int i = 0 ; i < mCurCharacter->getNumBodyNodes(); i++) {
			BodyInfo bi = mCurCharacterMap[i];
			mBodyPhaseMap.insert(std::pair<int, int>(i, bi.skelIdx));
		}
	}

	setJoinInfo();
}
void
ReferenceManager::
calculateHeightOffset()
{
	Eigen::VectorXd phase(mNumTimingFunc);
	phase.setZero();

	bool endFlag = false;
	bool noContact = true;
	double contactMin = 100;
	
	while(1) {
		Eigen::VectorXd p = getFrame(phase);
		mCurCharacter->setPositions(p);
		std::map<int, double> contacts = getContact(phase);

		for(auto it = contacts.begin(); it != contacts.end(); it++) {
			double h = mCurCharacter->getBodyNode(it->first)->getWorldTransform().translation()(1);
			if(it->second == 1)
				noContact = false;
			if(it->second == 1 && h < contactMin) {
				contactMin = h;
			}
		}

		phase += getTimeStep(phase);
		for(int i = 0; i < mNumTimingFunc; i++) {
			if(phase[i] >= getMotionLength(i)) {
				endFlag = true;
				break;
			}
		}
		if(endFlag)
			break;
	}
	if(!noContact) {
		double heightOffset; 
		if(contactMin < 0) {
			heightOffset -= contactMin;

		} else {
			heightOffset = -std::max(0.0, contactMin - 0.06);
		}
		int baseIdx = mBodyPhaseMap[0];
		for(int i =0 ; i < mBaseMotion[baseIdx].size(); i++) {
			mBaseMotion[baseIdx][i](4) += heightOffset;
		}
		for(int i =0 ; i < mBaseMotionGlobal[baseIdx].size(); i++) {
			mBaseMotionGlobal[baseIdx][i](4) += heightOffset;
		}
	}
}
Eigen::VectorXd
ReferenceManager::
getRandomStart()
{
	double r = mUniform(mMT);
	
	int n = (int) std::floor(mUniform(mMT)*mInitialTiming.size());
	return mInitialTiming[n];
	
}
Eigen::VectorXd 
ReferenceManager::
getTimeStep(Eigen::VectorXd _t)
{
	// Eigen::VectorXd timeStep(_t.rows());
	// for(int i = 0 ; i < _t.rows(); i++) {
	// 	double phase = std::fmod(_t[i], mMotionLength[i]);
			
	// 	int k0 = (int) std::floor(phase);
	// 	int k1 = (int) std::ceil(phase);	

	// 	double weight = phase - k0;
	// 	timeStep[i] = (1 - weight) * mTimeStep[i][k0] + weight * mTimeStep[i][k1];

	// }
	return mSpeed * mTimeStep;
}
void 
ReferenceManager::
setTimeStep()
{
	int lengthMax = mMotionLength[0];

	for(int i = 1; i < mNumTimingFunc; i++) {
		if(lengthMax < mMotionLength[i]) {
			lengthMax = mMotionLength[i];
		}
	}
	
	mTimeStep.resize(mNumTimingFunc);
	for(int i = 0; i < mNumTimingFunc; i++) {
		mTimeStep(i) = (double) mMotionLength[i] / lengthMax;
	}

	Eigen::VectorXd phase(mNumTimingFunc);
	phase.setZero();
	while(1){
		mInitialTiming.push_back(phase);
		bool end = false;
		phase += mTimeStep;
		for(int i = 0; i < mNumTimingFunc; i++) {
			if(phase(i) > mMotionLength[i] - mTimeStep(i) + 1e-4) {
				end = true;
				break;
			}
		}
		if(end)
			break;
	}

	// double lengthBase = Eigen::VectorXd::Ones(mNumTimingFunc).norm();

	// int lengthMax = mMotionLength[0];

	// for(int i = 1; i < mNumTimingFunc; i++) {
	// 	if(lengthMax < mMotionLength[i]) {
	// 		lengthMax = mMotionLength[i];
	// 	}
	// }
	// Eigen::VectorXd lastConst(mNumTimingFunc);
	// lastConst.setZero();

	// double length = 0;
	// std::vector<std::pair<Eigen::VectorXd, double>> constLength;
	// for(int i = 0; i < mConstraintSync.size(); i++) {
	// 	Eigen::VectorXd curConst(mNumTimingFunc);
	// 	for(int j = 0; j < mNumTimingFunc; j++) {		
	// 		curConst(j) = mConstraintSync[i](j) / getMotionLength(j);
	// 	}

	// 	if(i == 0) {
	// 		length += curConst.norm();
	// 		constLength.push_back(std::pair<Eigen::VectorXd, double>(mConstraintSync[i], curConst.norm()));
	// 	} else {
	// 		length += (curConst - lastConst).norm();
	// 		constLength.push_back(std::pair<Eigen::VectorXd, double>(mConstraintSync[i], (curConst - lastConst).norm()));
	// 	}
	// 	lastConst = curConst;
	// }
	// length += (Eigen::VectorXd::Ones(mNumTimingFunc) - lastConst).norm();

	// Eigen::VectorXd end(mNumTimingFunc);
	// for(int i = 0; i < mNumTimingFunc; i++)
	// 	end[i] = getMotionLength(i);

	// constLength.push_back(std::pair<Eigen::VectorXd, double>(end, (Eigen::VectorXd::Ones(mNumTimingFunc) - lastConst).norm()));

	// mTimeStep.clear();
	// for(int i = 0; i < mNumTimingFunc; i++) {
	// 	int count = 0;
	// 	mTimeStep.push_back(std::vector<double>());
	// 	for(int j = 0; j < getMotionLength(i) + 1; j++) {
	// 		if(constLength[count].first(i) < j && constLength.size() > count + 1) {
	// 			count += 1;
	// 		}
	// 		double time = constLength[count].second / length * lengthMax;
	// 		double distance;
	// 		if(count == 0) 
	// 			distance = constLength[count].first(i);
	// 		else
	// 			distance = constLength[count].first(i) - constLength[count-1].first(i);

	// 		mTimeStep[i].push_back(distance / time);		 	
	// 	}	
	// }


	// Eigen::VectorXd phase(mNumTimingFunc);
	// phase.setZero();
	// while(1){
	// 	mInitialTiming.push_back(phase);
	// 	bool end = false;
	// 	Eigen::VectorXd curPhase = phase;
	// 	for(int i = 0; i < mNumTimingFunc; i++) {
	// 		phase(i) += getTimeStep(curPhase)(i);
	// 		if(phase(i) > getMotionLength(i) - getTimeStep(curPhase)(i) + 1e-4) {
	// 			end = true;
	// 			break;
	// 		}
	// 	}
	// 	if(end)
	// 		break;
	// }

}
void
ReferenceManager::
setJoinInfo()
{
	mJoinInfo.clear();

	for(int i = 0; i < mCurCharacter->getNumBodyNodes(); i++) {
		if(i == 0) {
			JoinInfo ji;

			ji.level = 0;
			ji.rootIdx = 0;
			ji.rootBaseIdx = 0;

			ji.rootParentBaseIdx = -1;
			ji.rootParentIdx = -1;

			ji.phaseIdx = mBodyPhaseMap[i];
			
			mJoinInfo.push_back(ji);

		} else {
			auto bnParent = mCurCharacter->getBodyNode(i)->getParentBodyNode();
			int parentIdx = bnParent->getIndexInSkeleton();
			if(mBodyPhaseMap[parentIdx] != mBodyPhaseMap[i]) {
				JoinInfo ji;

				BodyInfo bi = mCurCharacterMap[i];

				ji.rootIdx = i;
				ji.rootBaseIdx = bi.skelBodyIdx;

				ji.rootParentIdx = parentIdx;

				if(bi.skelBodyIdx == 0){
					ji.rootParentBaseIdx = -1;
				} else {
					ji.rootParentBaseIdx = mSkels[bi.skelIdx]->getBodyNode(bi.skelBodyIdx)->getParentBodyNode()->getIndexInSkeleton();
				} 
				ji.phaseIdx = mBodyPhaseMap[i];

				bool levelFound = false;
				while(1) {
					for(int i = 0; i < mJoinInfo.size(); i++) {
						if(parentIdx == mJoinInfo[i].rootIdx) {
							ji.level = mJoinInfo[i].level + 1;
							levelFound = true;
							break;
						}
					}
					if(levelFound)
						break;

					parentIdx = mCurCharacter->getBodyNode(parentIdx)->getParentBodyNode()->getIndexInSkeleton();
				}

				mJoinInfo.push_back(ji);
			}
		}
	}

	for(int i = 0; i < mJoinInfo.size(); i++) {
		int idx = mJoinInfo[i].rootIdx;

		std::deque<int> bodyList;
		bodyList.push_back(idx);
		while(!bodyList.empty()) {
			idx = bodyList.front();
			mJoinInfo[i].bodyList.push_back(idx);

			bodyList.pop_front();

			int nChilds = mCurCharacter->getBodyNode(idx)->getNumChildBodyNodes();
			for(int j = 0; j < nChilds; j++) {
				int childIdx = mCurCharacter->getBodyNode(idx)->getChildBodyNode(j)->getIndexInSkeleton();
				bool nextBodyBlock = false;
				for(int k = 0; k < mJoinInfo.size(); k++) {
					if(mJoinInfo[k].rootIdx == childIdx) {
						nextBodyBlock = true;
						break;
					}
				}
				if(!nextBodyBlock) {
					bodyList.push_back(childIdx);
				}
			}
		}
	}
	

	//// debug
	for(int i = 0; i < mJoinInfo.size(); i++) {
		for(int j = 0; j < mJoinInfo[i].bodyList.size(); j++) {
			std::cout << i << " " << mCurCharacter->getBodyNode(mJoinInfo[i].bodyList[j])->getName() << std::endl;
		}
	}
}
std::pair<Eigen::VectorXd, Eigen::VectorXd> 
ReferenceManager::
getPositionWithDisplacement(Eigen::VectorXd _t,  Eigen::VectorXd _tprev, Eigen::VectorXd _prev, Eigen::VectorXd _displacement, int _id)
{
	Eigen::VectorXd pos = getFrame(_t, _tprev, _prev, _id);
	return std::pair<Eigen::VectorXd, Eigen::VectorXd> (pos, addDisplacement(pos, _displacement));
	
}
std::pair<Eigen::VectorXd, Eigen::VectorXd> 
ReferenceManager::
getPositionWithDisplacement(Eigen::VectorXd _t, Eigen::VectorXd _displacement, int _id)
{
	Eigen::VectorXd pos = getFrame(_t, _id);
	return std::pair<Eigen::VectorXd, Eigen::VectorXd> (pos, addDisplacement(pos, _displacement));

}
void
ReferenceManager::
saveCurrentParams(std::vector<Eigen::VectorXd> _param) 
{
	mLock.lock();
	for(int i = 0; i < _param.size(); i++) {
		mRecordCurParams.push_back(_param[i]);
	}
	mLock.unlock();
}
std::vector<Eigen::VectorXd> 
ReferenceManager::
buildParamTree() 
{
	mCurParamTree->clear();

	for(int i = 0; i < mRecordCurParams.size(); i++) {
		mCurParamTree->insertNode(new ParamNode(mRecordCurParams[i]), true);
	}
	mRecordCurParams.clear();
	std::vector<ParamNode*> paramNodes = mCurParamTree->traverse();

	std::vector<Eigen::VectorXd> params;
	for(int i = 0; i < paramNodes.size(); i++) {
		params.push_back(paramNodes[i]->getParam());
	}
	return params;
}
bool 
ReferenceManager::
optimizeReference()
{
	int optimized = 0;
	std::vector<ParamNode*> paramNodes = mCurParamTree->traverse();
	std::cout << "total params: " << paramNodes.size() << std::endl;
	for(int i = 0; i < paramNodes.size(); i++) {
		Eigen::VectorXd paramNormalized = paramNodes[i]->getParam();
	
		double d = mOptimizedParamTree->getDensity(paramNormalized);
		if(d < 0.2) {
			optimized += 1;
			Eigen::VectorXd ik = getPosSynthesized(paramNormalized);
			Eigen::VectorXd orig = getFrame(paramNormalized);
			Eigen::VectorXd d = subDisplacement(ik, orig);
			mOptimizedParamTree->insertNode(new ParamNode(paramNormalized, d));
		}
	}
	std::cout << "num optimized: " << optimized << std::endl;

	if(optimized)
		return true;
	else	
		return false;
}
void 
ReferenceManager::
pretrainReference()
{
	Eigen::VectorXd phase(mNumTimingFunc);
	phase.setZero();

	bool endFlag = false;
	while(1) {

		Eigen::VectorXd paramNormalized = phase;
		Eigen::VectorXd ik = getPosSynthesized(phase);

		Eigen::VectorXd orig = getFrame(phase);	
		Eigen::VectorXd d = subDisplacement(ik, orig);
		mOptimizedParamTree->insertNode(new ParamNode(paramNormalized, d, 1));
	
		phase += getTimeStep(phase);
		for(int i = 0; i < mNumTimingFunc; i++) {
			if(phase(i) >= getMotionLength(i)) {
				endFlag = true;
				break;
			}
		}
		if(endFlag)
			break;
	}

}
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> 
ReferenceManager::
getTrainingData()
{
	std::vector<ParamNode*> data = mOptimizedParamTree->traverse();
	std::vector<Eigen::VectorXd> x;
	std::vector<Eigen::VectorXd> y;
	for(int i = 0; i < data.size(); i++) {
		x.push_back(data[i]->getParam());
		y.push_back(data[i]->getDisplacement());
	}
	return std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> (x, y);
} 
Eigen::Vector3d
ReferenceManager::
getCurrentOrientation(dart::dynamics::SkeletonPtr _skel, int _idx, int _rootIdx)
{

	Eigen::Vector3d ori = Transform::getYrotation(_skel->getPositions().segment<3>(_rootIdx), mBaseAxisOrder[_idx]);
	return ori;
}
Eigen::VectorXd 
ReferenceManager::
getPosSynthesized(Eigen::VectorXd _t, bool _contact)
{
	return getPosSynthesized(_t, getStyle(_t), _contact);
}
Eigen::VectorXd 
ReferenceManager::
getPosSynthesized(Eigen::VectorXd _t, Eigen::VectorXd _weight, bool _contact)
{
	Eigen::VectorXd pos = getFrame(_t);
	if(mSkels.size() == 1 && mNumTimingFunc == 1) {
		return pos;
	}

	std::vector<std::tuple<int, int, Eigen::Vector3d>> constraints;
	std::vector<int> interestedBodies;
	
	for(int i = 0; i < mJoinInfo[0].bodyList.size(); i++) {
		interestedBodies.push_back(mJoinInfo[0].bodyList[i]);
	}
	int level = 1;
	while(1) {
		std::vector<int> optimDof;
		int count = 0;
		for(int i = 0; i < mJoinInfo.size(); i++) {
			if(mJoinInfo[i].level != level)
				continue;
			count += 1;
			
			int cSkelIdx = mCurCharacterMap[mJoinInfo[i].rootIdx].skelIdx;
			int phaseIdx = mJoinInfo[i].phaseIdx;
			auto bn = mCurCharacter->getBodyNode(mJoinInfo[i].rootIdx);

			double t = _t(phaseIdx);
			
			// get coordinate difference
			Eigen::VectorXd posBase = getBaseFrame(t, phaseIdx);
			mSkels[cSkelIdx]->setPositions(posBase);
			Eigen::Vector3d rootBase = getCurrentOrientation(mSkels[cSkelIdx], cSkelIdx);

			mCurCharacter->setPositions(pos);
			Eigen::Vector3d rootCur = getCurrentOrientation(mCurCharacter, mPhaseBaseMap[mBodyPhaseMap[0]]);

			Eigen::Matrix3d m = expMapRot(rootCur) * expMapRot(rootBase).transpose();	

			// get global orientation of base motion
			Eigen::VectorXd baseGlobal = getBaseFrame(t, phaseIdx, false, true);
			int idx = mSkels[cSkelIdx]->getBodyNode(mJoinInfo[i].rootBaseIdx)->getParentJoint()->getIndexInSkeleton(0);
			
			// calculate target
			Eigen::Vector3d targetOrientation = logMap(m * expMapRot(baseGlobal.segment<3>(idx)));

			targetOrientation = logMap(weightedSumRot(expMapRot(targetOrientation), bn->getTransform().linear(),  _weight[phaseIdx]));
			constraints.push_back(std::tuple<int, int, Eigen::Vector3d>(mJoinInfo[i].rootIdx, 1, targetOrientation));
		
			for(int j = 0; j < mJoinInfo[i].bodyList.size(); j++) {
				interestedBodies.push_back(mJoinInfo[i].bodyList[j]);
			}

			// add neighbor nodes for optimization dof
			idx = bn->getParentJoint()->getIndexInSkeleton(0);
			for(int j = 0; j < bn->getParentJoint()->getNumDofs(); j++)
				optimDof.push_back(idx+j);

			auto bnParent = bn->getParentBodyNode();
			if(bnParent) {
				int idx = bnParent->getParentJoint()->getIndexInSkeleton(0);
				if(idx == 0)
					continue;
				for(int j = 0; j < bnParent->getParentJoint()->getNumDofs(); j++)
					optimDof.push_back(idx+j);
			}
			
			if(bnParent->getParentBodyNode()) {
				int idx = bnParent->getParentBodyNode()->getParentJoint()->getIndexInSkeleton(0);
				if(idx == 0)
					continue;
				for(int j = 0; j < bnParent->getParentBodyNode()->getParentJoint()->getNumDofs(); j++)
					optimDof.push_back(idx+j);
			}
		}
		if(count == 0)
			break;
		MotionProcessor::IKcmaes(pos, optimDof, constraints, mCurCharacter, interestedBodies);
		level += 1;

	}

	// 	auto constFoot = getFootConstraints(pos, _t);
	// 	MotionProcessor::IKcmaes(pos, optimDof, constFoot, mCurCharacter);

	return pos;
}

Eigen::VectorXd 
ReferenceManager::
getPosSynthesized(Eigen::VectorXd _t, Eigen::VectorXd _tPrev, Eigen::VectorXd _prev)
{
	Eigen::VectorXd posSynthesized = getPosSynthesized(_t);
	Eigen::VectorXd posRaw = getFrame(_t);

	Eigen::Isometry3d isoSynThesized = dart::dynamics::FreeJoint::convertToTransform(posSynthesized.segment<6>(0));
	Eigen::Isometry3d isoRaw = dart::dynamics::FreeJoint::convertToTransform(posRaw.segment<6>(0));

	Eigen::Isometry3d isoDiff = isoRaw.inverse() * isoSynThesized;

	Eigen::VectorXd posCurRaw = getFrame(_t, _tPrev, _prev);
	Eigen::Isometry3d isoCurRaw = dart::dynamics::FreeJoint::convertToTransform(posCurRaw.segment<6>(0));	

	Eigen::VectorXd posCur = posSynthesized;
	Eigen::Isometry3d isoCur = isoCurRaw * isoDiff;

	posCur.segment<3>(0) = logMap(isoCur.linear());
	posCur.segment<3>(3) = isoCur.translation();

	return posCur;
}
std::vector<std::tuple<int, int, Eigen::Vector3d>>
ReferenceManager::
getFootConstraints(Eigen::VectorXd& _pos, Eigen::VectorXd _t, std::vector<int> _interestedBodies, bool _smooth)
{
	std::vector<std::tuple<int, int, Eigen::Vector3d>> constraints;
	if(_interestedBodies.size() == 0) {
		for(int j = 0; j < mCurCharacter->getNumBodyNodes(); j++) {
			_interestedBodies.push_back(j);
		}
	}

	if(_smooth) {
		int smoothRange = 4;
		double contactTh = 0.0;
		std::vector<std::pair<std::string, int>> idxs;
		std::vector<bool> needOptimization;
		
		for(int j = 0; j < _interestedBodies.size(); j++) {
			std::string name = mCurCharacter->getBodyNode(_interestedBodies[j])->getName();
			if(name.find("Foot") != std::string::npos) {
				idxs.push_back(std::pair<std::string, int>(name, mCurCharacter->getBodyNode(_interestedBodies[j])->getIndexInSkeleton()));
			}
			
		}
		
		mCurCharacter->setPositions(_pos);
		
		for(int i = 0; i < idxs.size(); i++) {
			needOptimization.push_back(false);

			int idx = idxs[i].second;

			Eigen::VectorXd phase = _t;
		
			std::map<int, double> contactMap = getContact(phase);
		
			Eigen::Vector3d target = mCurCharacter->getBodyNode(idx)->getTransform().translation();
			if(contactMap[idx] == 1 && target(1) > contactTh) {
				// contact point
				double heightDiff = target(1) - contactTh;

				target(1) = contactTh;
				constraints.push_back(std::tuple<int, int, Eigen::Vector3d>(idx, 0, target));

			} else {
				// check if current phase is near contact point

				int stepFutureContact = smoothRange + 1;
				double hDiffFuture = 0;
				int stepPastContact = smoothRange + 1;
				double hDiffPast = 0;

				bool endFlag = false;
				for(int j = 1; j <= smoothRange; j++) {
					phase += getTimeStep(phase);
					for(int k = 0; k < mNumTimingFunc; k++) {
						if(phase(k) >= getMotionLength(k)) {
							endFlag = true;
							break;
						}
					}
					if(endFlag)
						break;

					contactMap = getContact(phase);
					if(contactMap[idx] == 1) {
						stepFutureContact = j;
						
						Eigen::VectorXd pos = getFrame(phase);
						mCurCharacter->setPositions(pos);
						hDiffFuture = mCurCharacter->getBodyNode(idx)->getTransform().translation()(1) - contactTh;
						break;
					}
				}
				phase = _t;
				endFlag = false;
				for(int j = 1; j <= smoothRange; j++) {
					phase -= getTimeStep(phase);
					for(int k = 0; k < mNumTimingFunc; k++) {
						if(phase(k) < 0) {
							endFlag = true;
							break;
						}
					}

					if(endFlag || j >= stepFutureContact)
						break;
					contactMap = getContact(phase);

					if(contactMap[idx] == 1) {
						stepPastContact = j;

						Eigen::VectorXd pos = getFrame(phase);
						mCurCharacter->setPositions(pos);
						hDiffPast = mCurCharacter->getBodyNode(idx)->getTransform().translation()(1) - contactTh;
						break;
					}
				}
				if(stepFutureContact != smoothRange + 1 || stepPastContact != smoothRange + 1) {
					mCurCharacter->setPositions(_pos);

					Eigen::Vector3d target = mCurCharacter->getBodyNode(idx)->getTransform().translation();
				
					int phaseIdx = mBodyPhaseMap[idx];
					int skelIdx = mCurCharacterMap[idx].skelIdx;
					int skelBodyIdx = mCurCharacterMap[idx].skelBodyIdx;

					Eigen::VectorXd basePos = getBaseFrame(_t(phaseIdx), phaseIdx);
					mSkels[skelIdx]->setPositions(basePos);
					double hTarget = mSkels[skelIdx]->getBodyNode(skelBodyIdx)->getTransform().translation()(1);
					hTarget += (contactTh);
					
					double weight = 0;
					if(stepFutureContact < stepPastContact) {
						weight = (double) stepFutureContact / (smoothRange + 1);
					} else {
						weight = (double) stepPastContact / (smoothRange + 1);
					}
					double weightedTarget = (1 - weight) * hTarget + weight * target(1);
					target(1) = weightedTarget;
					if(target(1) < contactTh)
						target(1) = contactTh;
					constraints.push_back(std::tuple<int, int, Eigen::Vector3d>(idx, 0, target));
				
				}
			}
		}
	} else {			
		mCurCharacter->setPositions(_pos);

		for(int j = 0; j < _interestedBodies.size(); j++) {
			std::string name = mCurCharacter->getBodyNode(_interestedBodies[j])->getName();
  			int phaseIdx = mBodyPhaseMap[_interestedBodies[j]];
			int skelIdx = mCurCharacterMap[_interestedBodies[j]].skelIdx;
			int skelBodyIdx = mCurCharacterMap[_interestedBodies[j]].skelBodyIdx;

			int eeJoint = -1;
			if(name.find("Foot") != std::string::npos) {
				eeJoint = mSkels[skelIdx]->getBodyNode(skelBodyIdx)->getIndexInSkeleton();
			} 

			if(eeJoint != -1) {
				Eigen::VectorXd posBase = getBaseFrame(_t(phaseIdx), skelIdx);
				mSkels[skelIdx]->setPositions(posBase);
				double h = mSkels[skelIdx]->getBodyNode(eeJoint)->getTransform().translation()(1);
				Eigen::Vector3d trans = mCurCharacter->getBodyNode(_interestedBodies[j])->getTransform().translation();
				trans(1) = h;
				constraints.push_back(std::tuple<int, int, Eigen::Vector3d>(_interestedBodies[j], 0, trans));
			}
		}
	}
	
	return constraints;
}
void 
ReferenceManager::
addConstraint(Eigen::VectorXd _phase, Eigen::VectorXd _style, double _speed,
			  std::vector<bool> _activatePhase, std::vector<bool> _activateStyle, bool _activateSpeed) {
	if(_activateSpeed) {
		mSpeed = _speed;
	}
	for(int i = 0; i < _activateStyle.size(); i++) {
		if(_activateStyle[i]) {
			if(_activatePhase[i]) {
				mConstraintStyle.push_back(std::tuple<int, double, double>(i, _phase(i), _style(i)));
			} else {
				mBaseStyle(i) = _style(i);
			}
		}
	}

	bool activatePhaseAll = true;
	Eigen::VectorXd phase(mNumTimingFunc);
	for(int i = 0; i < _activatePhase.size(); i++) {
		if(_activatePhase[i]) {
			phase(i) = _phase(i);
		} else 
			activatePhaseAll = false;

	}
	if(activatePhaseAll) {
		mConstraintSync.push_back(phase);
		setTimeStep();
	}
}
Eigen::VectorXd 
ReferenceManager::
getStyle(Eigen::VectorXd _p) {
	Eigen::VectorXd style = mBaseStyle;
	for(int i = 0; i < mNumTimingFunc; i++) {
		_p(i) = std::fmod(_p(i), getMotionLength(i));
		int startIdx = -1;
		int endIdx = -1;
		for(int j = 0 ; j < mConstraintStyle.size(); j++) {
			if(std::get<0>(mConstraintStyle[j]) == i) {
				if(_p(i) > std::get<1>(mConstraintStyle[j]) && (startIdx == -1 || std::get<1>(mConstraintStyle[j]) > std::get<1>(mConstraintStyle[startIdx])))
					startIdx = j;
				if(_p(i) <= std::get<1>(mConstraintStyle[j]) && (endIdx == -1 || std::get<1>(mConstraintStyle[j]) < std::get<1>(mConstraintStyle[endIdx])))
					endIdx = j;
			}
		}
		if(startIdx != -1 && endIdx != -1) {
			double a = _p(i) - std::get<1>(mConstraintStyle[startIdx]);
			double b = std::get<1>(mConstraintStyle[endIdx]) - _p(i);
			style(i) = a / (a+b) * std::get<2>(mConstraintStyle[endIdx]) + b / (a+b) * std::get<2>(mConstraintStyle[startIdx]);
		} else if(startIdx != -1) {
			double a = _p(i) - std::get<1>(mConstraintStyle[startIdx]);
			double b = getMotionLength(i) - _p(i);
			style(i) = a / (a+b) * style(i) + b / (a+b) * std::get<2>(mConstraintStyle[startIdx]);	
		} else if(endIdx != -1) {
			double a = _p(i);
			double b = std::get<1>(mConstraintStyle[endIdx]) - _p(i);
			style(i) = a / (a+b) * std::get<2>(mConstraintStyle[endIdx]) + b / (a+b) * style(i);
		}
	}
	return style;
}
int 
ReferenceManager::
getMotionLength(int _idx) 
{ 
	return mMotionLength[_idx]; 
}
Eigen::VectorXd 
ReferenceManager::
toSinusoidalParam(Eigen::VectorXd _param) {
	Eigen::VectorXd x;
	x.resize(mNumTimingFunc*2);
	for(int j = 0; j < mNumTimingFunc; j++) {
		x.segment<2>(2*j) = Eigen::Vector2d(sin(2 * M_PI * _param(j) / getMotionLength(j)), 
											cos(2 * M_PI * _param(j) / getMotionLength(j)));
	}
	return x;
}
std::vector<std::pair<double, Eigen::VectorXd>> 
ReferenceManager::
getSyncConstraints() {
	std::vector<std::pair<double, Eigen::VectorXd>> constSync;
	for(int i = 0; i < mConstraintSync.size(); i++) {
		double phase = 0;
		for(int j = 0; j < mNumTimingFunc; j++) {
			phase += mConstraintSync[i][j] / mMotionLength[j];
		}
		phase /= mNumTimingFunc;
		constSync.push_back(std::pair<double, Eigen::VectorXd>(phase, mConstraintSync[i]));
	}
	std::stable_sort(constSync.begin(), constSync.end(), [] (std::pair<double, Eigen::VectorXd> i,
															 std::pair<double, Eigen::VectorXd> j) { 
		return (i.first < j.first); 
	});

	return constSync;
}
}