#ifndef __SIM_SKELETON_BUILDER_H__
#define __SIM_SKELETON_BUILDER_H__
#include "dart/dart.hpp"

namespace SIM
{
class XMLNode
{
public:
	XMLNode(std::string _name, Eigen::Vector3d _offset);
	void setParent(XMLNode* _parent);
	void setBodyTranslation(Eigen::Vector3d _bt);
	void setSize(Eigen::Vector3d _size);
	XMLNode* getParent();
	std::string getName();
	Eigen::Vector3d getBodyTranslation();
	Eigen::Vector3d getJointTranslation();
	Eigen::Vector3d getSize();
private:
	XMLNode* mParent;
	std::string mName;
	Eigen::Vector3d mJTfromParentJoint;
	Eigen::Vector3d mBTfromJoint;
	Eigen::Vector3d mSize;
};
struct BodyInfo
{
	int skelIdx;
	int skelBodyIdx;
	double scale;
};
struct SkelInfo
{
	dart::dynamics::SkeletonPtr skel;
	std::map<int, BodyInfo> bodyMap;
};
class SkeletonBuilder
{
public:
	static void generateNewSkeleton(std::string _motion, std::string _path);
	static dart::dynamics::SkeletonPtr buildFromFile(std::string _xml);
	static SkelInfo buildBodyBlock(int _xmlIdx, std::string _xml, std::vector<std::string> _blocklist);
	static dart::dynamics::SkeletonPtr buildSkelWithoutRoot(std::string _xml);

	static dart::dynamics::SkeletonPtr assembleBodyBlocks(dart::dynamics::SkeletonPtr _skel0,  int _joinIdx0,
									    dart::dynamics::SkeletonPtr _skelBlock, std::map<int, BodyInfo> _bodyMap1, int _joinIdx1, 
									    std::string _skel1_path);
  	static std::pair<SkelInfo, std::map<int, Eigen::Vector3d>> assembleFromFile(std::string _path, std::vector<std::string> _skelPath);
  	static void recordSkelInfoToFile(std::string _path, SkelInfo _skelInfo, std::map<int, Eigen::Vector3d> _rotMap);
    static dart::dynamics::BodyNode* makeFreeJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition,
		bool _isBall=false);

	static dart::dynamics::BodyNode* makeBallJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition,
		bool _isLimitEnforced,
		Eigen::Vector3d _upperLimit,
		Eigen::Vector3d _lowerLimit);
	static dart::dynamics::BodyNode* makeWeldJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d& _jointPosition,
		Eigen::Isometry3d& _bodyPosition);
	static dart::dynamics::BodyNode* makePrismaticJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
	 	Eigen::Isometry3d& _jointPosition,
		Eigen::Isometry3d& _bodyPosition,
	 	Eigen::Vector3d& _axis);
	static void scaleBodyNode(std::vector<dart::dynamics::SkeletonPtr> _baseSkels, const dart::dynamics::SkeletonPtr& _skelTarget, 
				  			  std::map<int, BodyInfo> _bodyMap);
};
}

#endif