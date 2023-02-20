#ifndef __SIM_PARAM_TREE_H__
#define __SIM_PARAM_TREE_H__
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <queue>

namespace SIM
{

class ParamNode
{
public:
	ParamNode(Eigen::VectorXd _p);
	ParamNode(Eigen::VectorXd _p, Eigen::VectorXd _d);
	ParamNode(Eigen::VectorXd _p, Eigen::VectorXd _d, int _count);
	ParamNode* getLeft() { return mLeft; }
	ParamNode* getRight() { return mRight; }
	void setLeft(ParamNode* _l) { mLeft = _l; }
	void setRight(ParamNode* _r) { mRight = _r; }
	void copy(ParamNode* _p);
	Eigen::VectorXd getParam() { return mParam; }
	Eigen::VectorXd getDisplacement() { return mDisplacement; }
	int getNumCount() { return mCount; }
	void addNumCount() { mCount += 1; }
private:
	ParamNode* mLeft;
	ParamNode* mRight;

	Eigen::VectorXd mParam;
	Eigen::VectorXd mDisplacement;
	int mCount;
};
struct distComp
{ 
	bool operator()(std::pair<double, ParamNode*> a, std::pair<double, ParamNode*> b) {
	    return a.first < b.first;
	}
};
class ParamTree
{
public:
	ParamTree(int _dim, Eigen::VectorXd _unit);
	void insertNode(ParamNode* _new, bool _checkNearest=false);
	double getDensity(Eigen::VectorXd _p);
	std::vector<std::pair<double, ParamNode*>> getNeighborNodes(Eigen::VectorXd _p, double _radius);
	std::vector<std::pair<double, ParamNode*>> getKNearestNodes(Eigen::VectorXd _p, int _n);
	std::vector<ParamNode*> traverse();
	std::vector<ParamNode*> traverseAndRebuild();
	void build(std::vector<ParamNode*> _params);
	double getDistance(Eigen::VectorXd _p0, Eigen::VectorXd _p1);
	void clear();
private:
	void insertRecursive(ParamNode* _root, ParamNode* _new, int _depth);
	ParamNode* findMinRecursive(ParamNode* _root, int _idx, int _depth);
	void getNeighborNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, double _radius, int _depth, std::vector<std::pair<double, ParamNode*>>& _results);
	void getKNearestNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, int _n, int _depth, 
								   std::priority_queue<std::pair<double, ParamNode*>, std::vector<std::pair<double, ParamNode*>>, distComp>& _heap);
	void traverseRecursive(ParamNode* _root, std::vector<ParamNode*>& _results);
	ParamNode* buildRecursive(int _depth, std::vector<ParamNode*> _params);
	bool isCorrectTree(ParamNode* _root, int _depth);

	ParamNode* mRoot;	
	Eigen::VectorXd mUnit;
	int mDim;
};
}
#endif