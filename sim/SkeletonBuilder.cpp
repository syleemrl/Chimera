#include <tinyxml2.h>
#include <fstream>
#include <cmath>
#include <queue>
#include "Functions.h"
#include "SkeletonBuilder.h"
#include "BVHParser.h"

typedef tinyxml2::XMLElement TiXmlElement;
typedef tinyxml2::XMLDocument TiXmlDocument;

#define JOINT_DAMPING 0.05

namespace SIM
{
XMLNode::
XMLNode(std::string _name, Eigen::Vector3d _offset)
:mJTfromParentJoint(_offset), mName(_name)
{}
void 
XMLNode::
setParent(XMLNode* _parent)
{
	mParent = _parent;
}
void 
XMLNode::
setBodyTranslation(Eigen::Vector3d _bt)
{
	mBTfromJoint = _bt;
}
void 
XMLNode::
setSize(Eigen::Vector3d _size)
{
	mSize = _size;	
}
XMLNode* 
XMLNode::
getParent()
{
	return mParent;
}
std::string 
XMLNode::
getName()
{
	return mName;
}
Eigen::Vector3d 
XMLNode::
getBodyTranslation()
{
	return mBTfromJoint;
}
Eigen::Vector3d 
XMLNode::
getJointTranslation()
{
	return mJTfromParentJoint;
}
Eigen::Vector3d 
XMLNode::
getSize()
{
	return mSize;
}
std::vector<XMLNode*>
generateXMLnodesRecursive(BVHNode* _current)
{
	std::vector<XMLNode*> bodynodes;
	XMLNode* bn = new XMLNode(_current->getName(), _current->getOffset());
	bodynodes.push_back(bn);
	
	std::vector<Eigen::Vector3d> childOffsets;
	for(int i = 0; i < _current->getChildren().size(); i++) {
		BVHNode* child = _current->getChildren()[i];
		childOffsets.push_back(child->getOffset());

		if(!child->isEnd()) {
			std::vector<XMLNode*> bodynodesChild = generateXMLnodesRecursive(child);
			bodynodesChild[0]->setParent(bn);
			for(int j = 0; j < bodynodesChild.size(); j++) {
				bodynodes.push_back(bodynodesChild[j]);
			}
		}
	}

	Eigen::Vector3d flag;
	Eigen::Vector3d min;
	Eigen::Vector3d max;

	flag.setZero();
	min.setZero();
	max.setZero();

	for(int i = 0; i < childOffsets.size(); i++) {
		for(int j = 0; j < 3; j++) {
			if(childOffsets[i][j] != 0) {
				if(childOffsets[i][j] < min[j]) {
					min[j] = childOffsets[i][j];
				} else if(childOffsets[i][j] > max[j]) {
					max[j] = childOffsets[i][j];
				}
				flag[j] = 1;
			}
		}
	}

	Eigen::Vector3d size = - min + max;
	for(int j = 0; j < 3; j++) {
		if(flag(j) == 0 || size(j) < 0.04) {
			min(j) += -0.06;
			max(j) += 0.06;
			size(j) = - min(j) + max(j);
		}
	}
	Eigen::Vector3d bodyTranslation = min + size / 2.0;
	bn->setSize(size);
	bn->setBodyTranslation(bodyTranslation);

	return bodynodes;
}
void 
SkeletonBuilder::
generateNewSkeleton(std::string _motion, std::string _path) 
{
	//torque limits and joint pos limits should be written manually.
	if(boost::filesystem::exists(_path)) {
		std::cout << "file exists. generation stopped." << std::endl;
		return;
	}
	BVHNode* root = BVHParser::parseHierarchy(_motion);
	std::vector<XMLNode*> bodynodes = generateXMLnodesRecursive(root);
	
	std::ofstream ofs(_path);
	ofs << "<Skeleton name=\"Humanoid\">" << std::endl;
	for(int i = 0; i < bodynodes.size(); i++) {
		if(i == 0) {
			ofs << "<Joint type=\"FreeJoint\" name=\"" 
				<< bodynodes[i]->getName() << "\" parent_name=\"None\" size=\"" 
				<< vec2Str(bodynodes[i]->getSize()) << "\" mass=\"5\">" << std::endl;
			ofs << "<BodyPosition translation=\""
				<< vec2Str(bodynodes[i]->getBodyTranslation()) << "\" />" << std::endl;
			ofs << "<JointPosition translation=\"0 0 0\" />" << std::endl;
		} else {
			ofs << "<Joint type=\"BallJoint\" name=\"" 
				<< bodynodes[i]->getName() << "\" parent_name=\""
				<< bodynodes[i]->getParent()->getName() << "\" size=\"" 
				<< vec2Str(bodynodes[i]->getSize()) << "\" mass=\"5\">" << std::endl;
			ofs << "<BodyPosition translation=\""
				<< vec2Str(bodynodes[i]->getBodyTranslation()) << "\" />" << std::endl;
			ofs << "<JointPosition translation=\"" 
				<< vec2Str(bodynodes[i]->getJointTranslation()) << "\" />" << std::endl;
		}
		ofs << "</Joint>" << std::endl;
	}
	ofs << "</Skeleton>" << std::endl;
	ofs.close();
}
dart::dynamics::SkeletonPtr
SkeletonBuilder::
buildFromFile(std::string _xml) {
	TiXmlDocument doc;
	if(doc.LoadFile(_xml.c_str())){
		std::cout << "Can't open file : " << _xml << std::endl;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	
	std::string skelname = skeldoc->Attribute("name");
	dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(skelname);


	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		std::string name = body->Attribute("name");
		std::string parentName = body->Attribute("parent_name");
		dart::dynamics::BodyNode *parent;
		if(!parentName.compare("None"))
			parent = nullptr;
		else
			parent = skel->getBodyNode(parentName);
		// size
		Eigen::Vector3d size = str2Vec3d(std::string(body->Attribute("size")));

		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		if(bodyPosElem->Attribute("linear")!=nullptr)
			bodyPosition.linear() = orthonormalize(str2Mat3d(bodyPosElem->Attribute("linear")));
		bodyPosition.translation() = str2Vec3d(bodyPosElem->Attribute("translation"));

		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = orthonormalize(str2Mat3d(jointPosElem->Attribute("linear")));
		jointPosition.translation() = str2Vec3d(jointPosElem->Attribute("translation"));

		// mass
		double mass = atof(body->Attribute("mass"));
		
		if(!jointType.compare("FreeJoint") ){
			SkeletonBuilder::makeFreeJointBody(skel, parent,
											   name, size, mass,
											   jointPosition, bodyPosition);
		} else if(!jointType.compare("BallJoint")){
			// joint limit
			bool isLimitEnforced = false;
			Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				isLimitEnforced = true;
				upperLimit = str2Vec3d(jointPosElem->Attribute("upper"));
			}
			if(jointPosElem->Attribute("lower")!=nullptr)
			{
				isLimitEnforced = true;
				lowerLimit = str2Vec3d(jointPosElem->Attribute("lower"));
			}

			SkeletonBuilder::makeBallJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition,
											   isLimitEnforced, upperLimit, lowerLimit);
		} else if(!jointType.compare("WeldJoint")){
			SkeletonBuilder::makeWeldJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition);			
		}  else if(!jointType.compare("PrismaticJoint")){
			Eigen::Vector3d axis = str2Vec3d(std::string(body->Attribute("axis")));

			SkeletonBuilder::makePrismaticJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition, axis);			
		}
	}
	return skel;
}
dart::dynamics::BodyNode* 
SkeletonBuilder::
makeFreeJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d _jointPosition,
	Eigen::Isometry3d _bodyPosition,
	bool _isBall)
{
	dart::dynamics::ShapePtr shape;
	if(_isBall)
		shape = std::shared_ptr<dart::dynamics::SphereShape>(new dart::dynamics::SphereShape(_size(0)));
	else
		shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::FreeJoint::Properties props;
	props.mName = _name;
	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;
	dart::dynamics::JointPtr jn = bn->getParentJoint();
	for(int i = 0; i < jn->getNumDofs(); i++){
		jn->getDof(i)->setDampingCoefficient(0.05);
	}
	bn->createShapeNodeWith<dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);	
	bn->setInertia(inertia);

	return bn;
}

dart::dynamics::BodyNode* 
SkeletonBuilder::
makeBallJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d _jointPosition,
	Eigen::Isometry3d _bodyPosition,
	bool _isLimitEnforced,
	Eigen::Vector3d _upperLimit,
	Eigen::Vector3d _lowerLimit)
{
	dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));
	
	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::BallJoint::Properties props;
	props.mName = _name;
	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::BallJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;

	dart::dynamics::JointPtr jn = bn->getParentJoint();
	for(int i = 0; i < jn->getNumDofs(); i++){
		jn->getDof(i)->setDampingCoefficient(JOINT_DAMPING);
	}

	bn->createShapeNodeWith<dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
	bn->setInertia(inertia);

	if(_isLimitEnforced){
		dart::dynamics::JointPtr joint = bn->getParentJoint();
		joint->setLimitEnforcement(_isLimitEnforced);

		for(int i = 0; i < 3; i++)
		{
			joint->setPositionUpperLimit(i, _upperLimit[i]);
			joint->setPositionLowerLimit(i, _lowerLimit[i]);
		}
	}

	return bn;
}
dart::dynamics::BodyNode* 
SkeletonBuilder::
makeWeldJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d& _jointPosition,
	Eigen::Isometry3d& _bodyPosition)
{
	dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::FreeJoint::Properties props;
	props.mName = _name;
	
	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;
	
	bn->createShapeNodeWith<dart::dynamics::VisualAspect,dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->setInertia(inertia);

	return bn;
}
SkelInfo 
SkeletonBuilder::
buildBodyBlock(int _xmlIdx, std::string _xml, std::vector<std::string> _blocklist)
{
	TiXmlDocument doc;
	SkelInfo result;

	if(doc.LoadFile(_xml.c_str())){
		std::cout << "Can't open file : " << _xml << std::endl;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	
	std::string skelname = skeldoc->Attribute("name") + std::string("Block");
	dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(skelname);
	result.skel = skel;

	bool blockRootFound = false;
	int bodyIdx = 0;
	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		std::string jointType = body->Attribute("type");
		std::string name = body->Attribute("name");
		std::string parentName = body->Attribute("parent_name");
		
		dart::dynamics::BodyNode *parent;
		if(!parentName.compare("None"))
			parent = nullptr;
		else
			parent = skel->getBodyNode(parentName);
		// size
		Eigen::Vector3d size = str2Vec3d(std::string(body->Attribute("size")));

		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		if(bodyPosElem->Attribute("linear")!=nullptr)
			bodyPosition.linear() = orthonormalize(str2Mat3d(bodyPosElem->Attribute("linear")));
		bodyPosition.translation() = str2Vec3d(bodyPosElem->Attribute("translation"));

		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = orthonormalize(str2Mat3d(jointPosElem->Attribute("linear")));
		jointPosition.translation() = str2Vec3d(jointPosElem->Attribute("translation"));
		// mass
		double mass = atof(body->Attribute("mass"));

		auto it = std::find(_blocklist.begin(), _blocklist.end(), name);
		auto itParent = std::find(_blocklist.begin(), _blocklist.end(), parentName);

		if(itParent ==  _blocklist.end() && it != _blocklist.end() && !blockRootFound) {
			//root joint
			blockRootFound = true;
			auto bn = SkeletonBuilder::makeFreeJointBody(skel, parent,
											   name, size, mass,
											   jointPosition, bodyPosition);

			BodyInfo curBodyInfo;
			curBodyInfo.skelIdx = _xmlIdx;
			curBodyInfo.skelBodyIdx = bodyIdx;

			result.bodyMap.insert(std::pair<int, BodyInfo>(skel->getIndexOf(bn), curBodyInfo));
		} else if(itParent !=  _blocklist.end()  && it != _blocklist.end() && blockRootFound) {
			// joint limit
			bool isLimitEnforced = false;
			Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				isLimitEnforced = true;
				upperLimit = str2Vec3d(jointPosElem->Attribute("upper"));
			}
			if(jointPosElem->Attribute("lower")!=nullptr)
			{
				isLimitEnforced = true;
				lowerLimit = str2Vec3d(jointPosElem->Attribute("lower"));
			}

			auto bn = SkeletonBuilder::makeBallJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition,
											   isLimitEnforced, upperLimit, lowerLimit);			
			BodyInfo curBodyInfo;
			curBodyInfo.skelIdx = _xmlIdx;
			curBodyInfo.skelBodyIdx = bodyIdx;
			result.bodyMap.insert(std::pair<int, BodyInfo>(skel->getIndexOf(bn), curBodyInfo));
		}
		bodyIdx += 1;
	}
	return result;
}
dart::dynamics::BodyNode* 
SkeletonBuilder::
makePrismaticJointBody(
	const dart::dynamics::SkeletonPtr& _skel,
	dart::dynamics::BodyNode* const _parent,
	std::string _name,
	Eigen::Vector3d _size,
	double _mass,
	Eigen::Isometry3d& _jointPosition,
	Eigen::Isometry3d& _bodyPosition,
	Eigen::Vector3d& _axis)
{
	dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(_size));

	dart::dynamics::Inertia inertia;
	inertia.setMass(_mass);
	inertia.setMoment(shape->computeInertia(_mass));

	dart::dynamics::BodyNode* bn;
	dart::dynamics::PrismaticJoint::Properties props;
	props.mName = _name;
	props.mAxis = _axis;

	if(_parent!=nullptr)
		props.mT_ParentBodyToJoint = _parent->getParentJoint()->getTransformFromChildBodyNode()*_jointPosition;
	props.mT_ChildBodyToJoint = _bodyPosition.inverse();
	// props.mActuatorType = dart::dynamics::Joint::LOCKED;

	bn = _skel->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(
		_parent, props, dart::dynamics::BodyNode::AspectProperties(_name)).second;
	
	bn->createShapeNodeWith<dart::dynamics::VisualAspect,dart::dynamics::CollisionAspect,dart::dynamics::DynamicsAspect>(shape);
	bn->setInertia(inertia);

	return bn;
}
dart::dynamics::SkeletonPtr
SkeletonBuilder:: 
assembleBodyBlocks(dart::dynamics::SkeletonPtr _skel0, int _joinIdx0,
				   dart::dynamics::SkeletonPtr _skelBlock, std::map<int, BodyInfo> _bodyMap1, int _joinIdx1,
				   std::string _skel1_path)
{
	dart::dynamics::SkeletonPtr skel1_without_root = buildSkelWithoutRoot(_skel1_path);
	
	dart::dynamics::SkeletonPtr mainSkel = _skel0;
	int mainBodyIdx = _joinIdx0;

	dart::dynamics::SkeletonPtr addSkelOriginal = buildFromFile(_skel1_path);

	dart::dynamics::SkeletonPtr addSkel = skel1_without_root;
	std::map<int, BodyInfo> addBodyMap = _bodyMap1;
	int addBodyIdx = _joinIdx1;

	std::cout <<_skel1_path << std::endl;
	for(int i = 0; i < _skelBlock->getNumBodyNodes(); i++) {
		int idx = addBodyMap[i].skelBodyIdx;
		auto bn = addSkelOriginal->getBodyNode(idx);
		for(int j = 0; j < bn->getNumChildBodyNodes(); j++) {
			std::string name = bn->getChildBodyNode(j)->getName();
			if(!_skelBlock->getBodyNode(name)) {
				std::cout << name << std::endl;
				addSkel->getBodyNode(name)->remove();
				std::cout << name << std::endl;

			}
		}
	}

	addSkel->getBodyNode(addBodyIdx)->moveTo(mainSkel, mainSkel->getBodyNode(mainBodyIdx));

	return mainSkel;
}
dart::dynamics::SkeletonPtr
SkeletonBuilder::
buildSkelWithoutRoot(std::string _xml) {
	TiXmlDocument doc;
	if(doc.LoadFile(_xml.c_str())){
		std::cout << "Can't open file : " << _xml << std::endl;
	}

	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");

	std::string skelname = skeldoc->Attribute("name");
	dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(skelname);


	for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint")){
		// type
		std::string jointType = body->Attribute("type");
		std::string name = body->Attribute("name");
		std::string parentName = body->Attribute("parent_name");
		dart::dynamics::BodyNode *parent;
		if(!parentName.compare("None"))
			parent = nullptr;
		else
			parent = skel->getBodyNode(parentName);

		// size
		Eigen::Vector3d size = str2Vec3d(std::string(body->Attribute("size")));

		// body position
		TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
		Eigen::Isometry3d bodyPosition;
		bodyPosition.setIdentity();
		if(bodyPosElem->Attribute("linear")!=nullptr)
			bodyPosition.linear() = orthonormalize(str2Mat3d(bodyPosElem->Attribute("linear")));
		bodyPosition.translation() = str2Vec3d(bodyPosElem->Attribute("translation"));

		// joint position
		TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
		Eigen::Isometry3d jointPosition;
		jointPosition.setIdentity();
		if(jointPosElem->Attribute("linear")!=nullptr)
			jointPosition.linear() = orthonormalize(str2Mat3d(jointPosElem->Attribute("linear")));
		jointPosition.translation() = str2Vec3d(jointPosElem->Attribute("translation"));

		// mass
		double mass = atof(body->Attribute("mass"));

		// if(!jointType.compare("FreeJoint") ){
		// 	SkeletonBuilder::makeFreeJointBody(skel, parent,
		// 									   name, size, mass,
		// 									   jointPosition, bodyPosition);
		// } 

		if(!jointType.compare("BallJoint") || !jointType.compare("FreeJoint")){
			// joint limit
			bool isLimitEnforced = false;
			Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
			if(jointPosElem->Attribute("upper")!=nullptr)
			{
				isLimitEnforced = true;
				upperLimit = str2Vec3d(jointPosElem->Attribute("upper"));
			}
			if(jointPosElem->Attribute("lower")!=nullptr)
			{
				isLimitEnforced = true;
				lowerLimit = str2Vec3d(jointPosElem->Attribute("lower"));
			}
			SkeletonBuilder::makeBallJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition,
											   isLimitEnforced, upperLimit, lowerLimit);
		} else if(!jointType.compare("WeldJoint")){
			SkeletonBuilder::makeWeldJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition);			
		}
	}
	return skel;
}
std::pair<SkelInfo, std::map<int, Eigen::Vector3d>> 
SkeletonBuilder::
assembleFromFile(std::string _path, std::vector<std::string> _skelPath)
{
	SkelInfo result;
	std::map<int, BodyInfo> bodyMap;
	std::vector<std::pair<int, Eigen::Vector3d>> parentVec;

	std::ifstream ifs(_path);
	char buffer[256];

	ifs >> buffer;
	int n = atoi(buffer);
	for(int i = 0; i < n; i++) {
		ifs >> buffer;
		int idx = atoi(buffer);
		if(idx != i) {
			std::cout << "wrong index" << std::endl;
		}
		ifs >> buffer;
		int skelIdx = atoi(buffer);
		ifs >> buffer;
		int skelBodyIdx = atoi(buffer);	
		ifs >> buffer;
		double scale = atof(buffer);	

		BodyInfo bi;
		bi.skelIdx = skelIdx;
		bi.skelBodyIdx = skelBodyIdx;
		bi.scale = scale;

		bodyMap.insert(std::pair<int, BodyInfo>(i, bi));

		ifs >> buffer;
		int parentIdx = atoi(buffer);

		Eigen::Vector3d vec;
		ifs >> buffer;
		vec(0) = atof(buffer);
		ifs >> buffer;
		vec(1) = atof(buffer);
		ifs >> buffer;
		vec(2) = atof(buffer);
		
		parentVec.push_back(std::pair<int, Eigen::Vector3d>(parentIdx, vec));	
	}
	std::map<int, Eigen::Vector3d> rotMap;
	ifs >> buffer;
	n = atoi(buffer);
	for(int i = 0; i < n; i++) {
		ifs >> buffer;
		int idx = atoi(buffer);
		
		Eigen::Vector3d vec;
		ifs >> buffer;
		vec(0) = atof(buffer);
		ifs >> buffer;
		vec(1) = atof(buffer);
		ifs >> buffer;
		vec(2) = atof(buffer);
		rotMap.insert(std::pair<int, Eigen::Vector3d>(idx, vec));
	}

	std::string skelname = "Chimera";
	dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(skelname);
	
	result.bodyMap = bodyMap;
	result.skel = skel;

	int curBodyIdx = 0;
	int curSkelIdx = bodyMap[0].skelIdx;

	TiXmlDocument doc;
	if(doc.LoadFile(_skelPath[bodyMap[0].skelIdx].c_str())){
		std::cout << "Can't open file : " << _skelPath[bodyMap[0].skelIdx] << std::endl;
	}
	TiXmlElement* skeldoc = doc.FirstChildElement("Skeleton");

	int numBodyNodes = bodyMap.size();
	while(curBodyIdx < numBodyNodes) {
		int xmlBodyIdx = 0;

		for(TiXmlElement *body = skeldoc->FirstChildElement("Joint"); body != nullptr; body = body->NextSiblingElement("Joint"))
		{
			if(bodyMap.find(curBodyIdx) == bodyMap.end())
				break;

			if(bodyMap[curBodyIdx].skelIdx != curSkelIdx) {
				curSkelIdx = bodyMap[curBodyIdx].skelIdx;
				doc.LoadFile(_skelPath[bodyMap[curBodyIdx].skelIdx].c_str());
				skeldoc = doc.FirstChildElement("Skeleton");
				break;
			}

			if(bodyMap[curBodyIdx].skelBodyIdx == xmlBodyIdx) {
				std::string jointType = body->Attribute("type");
				std::string name = body->Attribute("name") + std::string("FromSkel") + std::to_string(bodyMap[curBodyIdx].skelIdx);
				int isExist = 0;
				for(int i = 0; i < skel->getNumBodyNodes(); i++) {
					if(skel->getBodyNode(i)->getName().find(name) != std::string::npos) {
						isExist += 1;
					}
				}
				if(isExist != 0)
					name += "_" + std::to_string(isExist + 1);

				dart::dynamics::BodyNode *parent = skel->getBodyNode(parentVec[curBodyIdx].first);
				// size
				Eigen::Vector3d size = str2Vec3d(std::string(body->Attribute("size")));

				// body position
				TiXmlElement *bodyPosElem = body->FirstChildElement("BodyPosition");
				Eigen::Isometry3d bodyPosition;
				bodyPosition.setIdentity();
				if(bodyPosElem->Attribute("linear")!=nullptr)
					bodyPosition.linear() = orthonormalize(str2Mat3d(bodyPosElem->Attribute("linear")));
				bodyPosition.translation() = str2Vec3d(bodyPosElem->Attribute("translation"));

				// joint position
				TiXmlElement *jointPosElem = body->FirstChildElement("JointPosition");
				Eigen::Isometry3d jointPosition;
				jointPosition.setIdentity();
				if(jointPosElem->Attribute("linear")!=nullptr)
					jointPosition.linear() = orthonormalize(str2Mat3d(jointPosElem->Attribute("linear")));
				jointPosition.translation() = str2Vec3d(jointPosElem->Attribute("translation"));
				// mass
				double mass = atof(body->Attribute("mass"));

				if(curBodyIdx == 0) {
					auto bn = SkeletonBuilder::makeFreeJointBody(skel, parent,
													   name, size, mass,
													   jointPosition, bodyPosition);
				} else {
					bool isLimitEnforced = false;
					Eigen::Vector3d upperLimit(1E6,1E6,1E6), lowerLimit(-1E6,-1E6,-1E6);
					if(jointPosElem->Attribute("upper")!=nullptr)
					{
						isLimitEnforced = true;
						upperLimit = str2Vec3d(jointPosElem->Attribute("upper"));
					}
					if(jointPosElem->Attribute("lower")!=nullptr)
					{
						isLimitEnforced = true;
						lowerLimit = str2Vec3d(jointPosElem->Attribute("lower"));
					}

					auto bn = SkeletonBuilder::makeBallJointBody(skel, parent, 
											   name, size, mass,
											   jointPosition, bodyPosition,
											   isLimitEnforced, upperLimit, lowerLimit);

					auto props = bn->getParentJoint()->getJointProperties();
					props.mT_ParentBodyToJoint.translation() = parentVec[curBodyIdx].second;
					bn->getParentJoint()->setProperties(props);
				} 

				curBodyIdx += 1;
			}
			xmlBodyIdx += 1;
		}
	}

	return std::pair<SkelInfo, std::map<int, Eigen::Vector3d>>(result, rotMap);
}
void 
SkeletonBuilder::
recordSkelInfoToFile(std::string _path, SkelInfo _skelInfo, std::map<int, Eigen::Vector3d> _rotMap)
{
	dart::dynamics::SkeletonPtr skel = _skelInfo.skel;
	std::map<int, BodyInfo> bodyMap = _skelInfo.bodyMap;

	std::ofstream ofs(_path);
	ofs << bodyMap.size() << std::endl;
	for(auto it=bodyMap.begin(); it != bodyMap.end(); it++) {
		int parentIdx = -1;
		Eigen::Vector3d posFromParent;
		posFromParent.setZero();

		if(it->first != 0) {
			auto bn = skel->getBodyNode(it->first)->getParentBodyNode();
			parentIdx = skel->getIndexOf(bn);
			
			auto props = skel->getBodyNode(it->first)->getParentJoint()->getJointProperties();
			posFromParent = props.mT_ParentBodyToJoint.translation();

		}
		
		ofs << it->first << " " << it->second.skelIdx << " " << it->second.skelBodyIdx << " " << it->second.scale 
		    << " " << parentIdx << " " << posFromParent.transpose() << " " << std::endl;
	}

	ofs << _rotMap.size() << std::endl;
	for(auto it=_rotMap.begin(); it != _rotMap.end(); it++) {
		ofs << it->first << " "<< it->second.transpose() << std::endl;
	}
	ofs.close();
}
void 
SkeletonBuilder::
scaleBodyNode(std::vector<dart::dynamics::SkeletonPtr> _baseSkels, const dart::dynamics::SkeletonPtr& _skelTarget, 
			  std::map<int, BodyInfo> _bodyMap) {
	
	for(int i = 0; i < _skelTarget->getNumBodyNodes(); i++) {

		double scale = _bodyMap[i].scale;
		auto bn = _skelTarget->getBodyNode(i);
		auto skelOriginal = _baseSkels[_bodyMap[i].skelIdx];
		auto bnOriginal = skelOriginal->getBodyNode(_bodyMap[i].skelBodyIdx);
		auto shapeOriginal = bnOriginal->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get();

		auto shapeOld = bn->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get();

		auto box = dynamic_cast<dart::dynamics::BoxShape*>(shapeOriginal);
		Eigen::Vector3d sizeOriginal = box->getSize();
		Eigen::Vector3d size = sizeOriginal * scale;

		auto inertia = bn->getInertia();
		inertia.setMass(bnOriginal->getMass() * scale);
			
		inertia.setMoment(shapeOld->computeInertia(inertia.getMass()));
		bn->setInertia(inertia);

		dart::dynamics::ShapePtr shape = std::shared_ptr<dart::dynamics::BoxShape>(new dart::dynamics::BoxShape(size));
			
		bn->removeAllShapeNodes();
		bn->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(shape);
			
		auto propsOriginal = bnOriginal->getParentJoint()->getJointProperties();
		Eigen::Vector3d CBtoJOriginal = propsOriginal.mT_ChildBodyToJoint.translation();
		Eigen::Vector3d PBtoJOriginal = propsOriginal.mT_ParentBodyToJoint.translation();
		
		auto props = bn->getParentJoint()->getJointProperties();
		bn->getParentJoint()->setProperties(props);
		props.mT_ChildBodyToJoint.translation() = CBtoJOriginal * scale;

		bool join = false;
		if(i!= 0) {
			auto bnParent = bn->getParentBodyNode();
			int idxParent = bnParent->getIndexInSkeleton();
			if(_bodyMap[idxParent].skelIdx != _bodyMap[i].skelIdx)
				join = true;
		}
		if(!join && scale != 1)
			props.mT_ParentBodyToJoint.translation() = PBtoJOriginal * scale;
		bn->getParentJoint()->setProperties(props);
	}
}
};