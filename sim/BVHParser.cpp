#include <iostream>
#include <fstream>
#include "BVHParser.h"
#include "MotionProcessor.h"
#include "Functions.h"
namespace SIM
{
Eigen::Vector3d BVHParser::mHipOffset;

BVHNode::
BVHNode(BVHNode* _parent, std::string _name) 
: mParent(_parent), mName(_name) 
{
	if(mParent)
		mIsRoot = false;
	else
		mIsRoot = true;
}	
BVHNode::
~BVHNode() 
{
	while(!mChildren.empty()) {
		BVHNode* c = mChildren.back();
		mChildren.pop_back();

		delete c;
	}
}
void 
BVHNode::
addChild(BVHNode* _child) 
{
	mChildren.push_back(_child);
}
void 
BVHNode::
setIsEnd(bool _isEnd) 
{
	mIsEndEffector = _isEnd;
}
void 
BVHNode::
setOffset(double _x, double _y, double _z) 
{
	mOffset << _x, _y, _z;
}
void 
BVHNode::
setChannel(std::vector<std::string> _ch) 
{
	mChannels = _ch;
}
std::vector<std::string> 
BVHNode::
getChannels() 
{
	return mChannels;
}
std::vector<BVHNode*> 
BVHNode::
getChildren()
{
	return mChildren;
}
Eigen::Vector3d 
BVHNode::
getOffset()
{
	return mOffset;
}
std::string 
BVHNode::
getName()
{
	return mName;
}
bool 
BVHNode::
isRoot()
{
	return mIsRoot;
}
bool 
BVHNode::
isEnd()
{
	return mIsEndEffector;
}
void
parseHierarchyRecursive(BVHNode* _current, std::ifstream& _is)
{
	char buffer[256];

	_is >> buffer; //{
	while(_is >> buffer)
	{
		std::string str = buffer;

		if(!strcmp(buffer,"}")) {
			break;
		}
		if(!strcmp(buffer,"OFFSET"))
		{
			double x,y,z;

			_is >> x;
			_is >> y;
			_is >> z;
			_current->setOffset(x*0.01, y*0.01, z*0.01);
		} else if(!strcmp(buffer,"CHANNELS"))
		{
			_is >> buffer;
			int n;
			n = atoi(buffer);
			std::vector<std::string> channels;
			if(_current->isRoot() && n != 6 && n != 9) {
				std::cout << "Supports 6 or 9 dof only for root joints" << std::endl;
			} 
			for(int i = 0; i < n; i++)
			{
				_is >> buffer;
				channels.push_back(std::string(buffer));
			}
			_current->setChannel(channels);

		} else if(!strcmp(buffer,"JOINT"))
		{
			_is >> buffer;
			BVHNode* child = new BVHNode(_current, std::string(buffer));
			child->setIsEnd(false);
			_current->addChild(child);

			parseHierarchyRecursive(child, _is);
		} else if(!strcmp(buffer,"End"))
		{
			_is >> buffer;
			BVHNode* child = new BVHNode(_current, _current->getName() + std::string("End"));
			child->setIsEnd(true);
			_current->addChild(child);

			parseHierarchyRecursive(child, _is);
		} 
	}
}
BVHNode* 
BVHParser::
parseHierarchy(std::string _bvh) 
{
	std::ifstream is(_bvh);
	char buffer[256];

	if(!is)
	{
		std::cout << "Can't open file: " << _bvh << std::endl;
		return 0;
	}
	BVHNode* root;
	while(is >> buffer)
	{
		if(!strcmp(buffer,"HIERARCHY"))
		{
			is>>buffer;//Root
			is>>buffer;//Name

			root = new BVHNode(0, std::string(buffer));
			root->setIsEnd(false);
			parseHierarchyRecursive(root, is);
			break;
		}
	}
	is.close();

	return root;
}
Eigen::VectorXd
parseFrameRecursive(BVHNode* _current, std::ifstream& _is)
{
	std::vector<std::string> ch = _current->getChannels();
	Eigen::VectorXd p(3);
	char buffer[256];
	int n = _current->getChannels().size();
	Eigen::VectorXd dof(n);

	for(int i = 0; i < n; i++) {
		_is >> buffer;
		dof(i) = atof(buffer);
	}
	Eigen::Vector3d vec;
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d trans;
	for(int i = 0; i < n; i++) {
		if(_current->getChannels()[i] == "Xrotation")
		{
			rot *= eulerX2Rot(dof(i));
			vec(1) = dof(i);
		} else if(_current->getChannels()[i] == "Yrotation")
		{
			rot *= eulerY2Rot(dof(i));
			vec(2) = dof(i);

		} else if(_current->getChannels()[i] == "Zrotation")
		{
			rot *= eulerZ2Rot(dof(i));
			vec(0) = dof(i);

		} else if(_current->getChannels()[i] == "Xposition")
		{
			trans(0) = dof(i)*0.01;
		} else if(_current->getChannels()[i] == "Yposition")
		{
			trans(1) = dof(i)*0.01;
		} else if(_current->getChannels()[i] == "Zposition")
		{
			trans(2) = dof(i)*0.01;
		}
	}
	// if(_current->getName() == "root_bone") {
	// 	BVHParser::mHipOffset(0) = trans(0);
	// 	BVHParser::mHipOffset(1) = trans(2);
	// 	BVHParser::mHipOffset(2) = trans(1);

	// }
	// if(_current->getName() == "Spine_base") {
	// 	trans(1) -= 1.42699;
	// 	BVHParser::mHipOffset += trans;
	// }
	if(_current->isRoot()) {
		p.resize(n);
		p.segment<3>(3) = logMap(rot);
		p.segment<3>(0) = trans;
	} else {
		p = logMap(rot);
	}

	for(int i = 0; i < _current->getChildren().size(); i++)
	{
		BVHNode* child = _current->getChildren()[i];
		if(!child->isEnd()) {
			Eigen::VectorXd pChild = parseFrameRecursive(_current->getChildren()[i], _is);	
			Eigen::VectorXd p_(p.rows() + pChild.rows());
			p_.head(p.rows()) = p;
			p_.tail(pChild.rows()) = pChild;
			p = p_;

		}
	}

	return p;
}
std::pair<double,std::vector<Eigen::VectorXd>>
BVHParser::
parseMotion(std::string _bvh, BVHNode* _root)
{
	std::ifstream is(_bvh);
	char buffer[256];
	double timestep;
	std::vector<Eigen::VectorXd> positions;
	while(is >> buffer)
	{
		if(!strcmp(buffer,"MOTION"))
		{
			is >> buffer; //Frames:
			is >> buffer; //num_frames
			int nFrames = atoi(buffer);
			is >> buffer; //Frame
			is >> buffer; //Time:
			is >> buffer; //time step
			timestep = atof(buffer);
			for(int i = 0; i < nFrames; i++)
			{
				// BVHParser::mHipOffset.setZero();
				Eigen::VectorXd p = parseFrameRecursive(_root, is);
				if(_bvh.find("horse_atkhead.bvh") != std::string::npos) {
					p(1) += 0.1;
				}
				// p.segment<3>(0) += BVHParser::mHipOffset;
				positions.push_back(p);
			}
		}
	}
	is.close();
	return std::pair<double,std::vector<Eigen::VectorXd>>(timestep, positions);
}
std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> 
BVHParser::
parse(std::string _bvh){
	BVHNode* root = parseHierarchy(_bvh);
	std::pair<double, std::vector<Eigen::VectorXd>> tp = parseMotion(_bvh, root);
	return std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>>(root, tp.first, tp.second);
}
std::vector<std::pair<std::string, int>> 
BVHParser::
getNodeIdx(BVHNode* _current) {
	std::vector<std::pair<std::string, int>> idxList;
	idxList.push_back(std::pair<std::string, int>(_current->getName(), 0));
	int idxMax = 0;
	if(_current->isRoot())
		idxMax = _current->getChannels().size();
	else
		idxMax = 3;
	for(int i = 0 ; i < _current->getChildren().size(); i++) {
		BVHNode* child = _current->getChildren()[i];
		if(!child->isEnd()) {
			std::vector<std::pair<std::string, int>> idxListChild = getNodeIdx(child);
			for(int j = 0; j < idxListChild.size(); j++) {
				idxListChild[j].second += idxMax;
				idxList.push_back(idxListChild[j]);
			}
			idxMax = idxList.back().second + 3;
		}
	}
	return idxList;
}
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> 
BVHParser::
loadPosAndVelFromBVH(std::string _bvh, dart::dynamics::SkeletonPtr _skel) {
	std::vector<Eigen::VectorXd> posVec;
	std::vector<Eigen::VectorXd> velVec;
	std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> bvhInfo = parse(_bvh);

	std::vector<std::pair<std::string, int>> idxList = getNodeIdx(std::get<0>(bvhInfo)); 

	double timeStep = std::get<1>(bvhInfo);
	int dof = _skel->getNumDofs();

	double t = 0;
	int count = 0;
	for(int i = 0; i < std::get<2>(bvhInfo).size(); i++)
	{
		bool flag = false;
		Eigen::VectorXd	p;
		if(abs(timeStep - TIME_STEP) <= 1e-4) {
			p = std::get<2>(bvhInfo)[i];
			flag = true;
		} else if(t >= count * TIME_STEP) {

			if(i != 0) {
				Eigen::VectorXd p0 = std::get<2>(bvhInfo)[i-1];
				Eigen::VectorXd p1 = std::get<2>(bvhInfo)[i];

				double t0 = t - timeStep;
				double t1 = t;

				p = weightedSumPos(p0, p1, (count * TIME_STEP - t0) / (t1 - t0));
			} else {
				p = std::get<2>(bvhInfo)[i];
			}
			flag = true;
		}

		if(flag) {
			Eigen::VectorXd pos(dof);

			for(int j = 0; j < idxList.size(); j++) {
				dart::dynamics::BodyNode* bn = _skel->getBodyNode(idxList[j].first);
				int idx = bn->getParentJoint()->getIndexInSkeleton(0);
				if(idx == 0) {
					pos.segment<3>(3) = p.segment<3>(0);
					pos.segment<3>(0) = p.segment<3>(3);
				} else {
					pos.segment<3>(idx) = p.segment<3>(idxList[j].second);
				}
			}
		
			Eigen::VectorXd vel = Eigen::VectorXd::Zero(dof);
			if(count >= 2) {
				vel = _skel->getPositionDifferences(pos, posVec.back()) / TIME_STEP;
			} else if(count == 1) {
				vel = _skel->getPositionDifferences(pos, posVec.back()) / TIME_STEP;
				velVec.back() = vel;
			}
			posVec.push_back(pos);
			velVec.push_back(vel);

			count += 1;

		}
		if(t < count * TIME_STEP)
			t += timeStep;
	}
	return std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> (posVec, velVec);
}
};