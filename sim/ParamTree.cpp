#include "ParamTree.h"
#include "Functions.h"
#include <iterator>
#include <algorithm>
#include <iostream>
namespace SIM
{
ParamNode::
ParamNode(Eigen::VectorXd _p) 
:mParam(_p), mCount(1), mLeft(0), mRight(0)
{}
ParamNode::
ParamNode(Eigen::VectorXd _p, Eigen::VectorXd _d) 
:mParam(_p), mDisplacement(_d), mCount(1), mLeft(0), mRight(0)
{}
ParamNode::
ParamNode(Eigen::VectorXd _p, Eigen::VectorXd _d, int _count) 
:mParam(_p), mDisplacement(_d), mCount(_count), mLeft(0), mRight(0)
{}
void 
ParamNode::
copy(ParamNode* _p)
{
	mParam = _p->getParam();
	mDisplacement = _p->getDisplacement();
	mCount = _p->getNumCount();
}	
ParamTree::
ParamTree(int _dim, Eigen::VectorXd _unit) : mDim(_dim), mRoot(0), mUnit(_unit)
{}
void
ParamTree::
insertNode(ParamNode* _new, bool _checkNearest)
{
	if(!mRoot) {
		mRoot = _new;
	}
	else {
		if(_checkNearest) {
			std::vector<std::pair<double, ParamNode*>> nearest = getKNearestNodes(_new->getParam(), 1);
			if(nearest[0].first < 0.5) {
				nearest[0].second->addNumCount();
				return;
			}
		}
		insertRecursive(mRoot, _new, 0);
	}
}
void
ParamTree::
insertRecursive(ParamNode* _root, ParamNode* _new, int _depth) {
	int idx = _depth % mDim;
	if(_root->getParam()[idx] > _new->getParam()[idx]) {
		if(!_root->getLeft()) 
			_root->setLeft(_new);
		else
			insertRecursive(_root->getLeft(), _new, _depth+1);
	} else {
		if(!_root->getRight()) 
			_root->setRight(_new);
		else
			insertRecursive(_root->getRight(), _new, _depth+1);
	}
}
bool
ParamTree::
isCorrectTree(ParamNode* _root, int _depth)
{
	if (_root == 0)
		return true;
	int idx = _depth % mDim;

	if(_root->getLeft() && _root->getParam()[idx] <= _root->getLeft()->getParam()[idx] )
		return false;
	else if(_root->getRight() && _root->getParam()[idx] > _root->getRight()->getParam()[idx])
		return false;

	return isCorrectTree(_root->getLeft(), _depth+1) && isCorrectTree(_root->getRight(), _depth+1);
}
double 
ParamTree::
getDensity(Eigen::VectorXd _p)
{
	double density = 0;
	std::vector<std::pair<double, ParamNode*>> nns = getNeighborNodes(_p, 2);
	for(int i = 0; i < nns.size(); i++) {
		density += nns[i].second->getNumCount() * exp(-pow(nns[i].first, 2));	
	}
	return density;
}
ParamNode*
ParamTree::
findMinRecursive(ParamNode* _root, int _idx, int _depth)
{
	if(_root == 0) {
		return 0;
	}

	int idx = _depth % mDim;
	if(idx == _idx) {
		if(!_root->getLeft()) {
			return _root;
		}
		else {
			return findMinRecursive(_root->getLeft(), _idx, _depth+1); 
		}
	}
		
	ParamNode* minLeft = findMinRecursive(_root->getLeft(), _idx, _depth+1);
	ParamNode* minRight = findMinRecursive(_root->getRight(), _idx, _depth+1);
	
	if(!minLeft && !minRight) {
		return _root;
	}

	else if(!minLeft) {
		if(_root->getParam()[_idx] <= minRight->getParam()[_idx]) {
			return _root;
		}
		
		return minRight;

	} else if(!minRight) {
		if(minLeft->getParam()[_idx] <= _root->getParam()[_idx])
			return minLeft;
		
		return _root;
	} else {
		if(minLeft->getParam()[_idx] <= _root->getParam()[_idx] &&
		   minLeft->getParam()[_idx] <= minRight->getParam()[_idx])
			return minLeft;

		if(_root->getParam()[_idx] <= minLeft->getParam()[_idx] &&
		   _root->getParam()[_idx] <= minRight->getParam()[_idx])
			return _root;

		return minRight;
	}
}
std::vector<std::pair<double, ParamNode*>>
ParamTree::
getNeighborNodes(Eigen::VectorXd _pn, double _radius)
{
	std::vector<std::pair<double, ParamNode*>> results;
	getNeighborNodesRecursive(mRoot, _pn, _radius, 0, results);
	return results;
}
void
ParamTree::
getNeighborNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, double _radius, int _depth, 
						  std::vector<std::pair<double, ParamNode*>>& _results)
{
	if (_root == 0)
		return;

	int idx = _depth % mDim;

	double dist = getDistance(_root->getParam(), _pn);
	if(dist <= _radius) {
		_results.push_back(std::pair<double, ParamNode*>(dist, _root));
	}

	Eigen::VectorXd projected = _pn;
	projected[idx] = _root->getParam()[idx];
	dist = getDistance(projected, _pn);
	if(dist <= _radius || _root->getParam()[idx] <= _pn[idx]) {
		getNeighborNodesRecursive(_root->getRight(), _pn, _radius, _depth+1, _results);
	}
	if(dist <= _radius || _root->getParam()[idx] > _pn[idx]) {
		getNeighborNodesRecursive(_root->getLeft(), _pn, _radius, _depth+1, _results);
	}
}
std::vector<std::pair<double, ParamNode*>>
ParamTree::
getKNearestNodes(Eigen::VectorXd _pn, int _n)
{
	std::priority_queue<std::pair<double, ParamNode*>, std::vector<std::pair<double, ParamNode*>>, distComp> heap;
	getKNearestNodesRecursive(mRoot, _pn, _n, 0, heap);

	std::vector<std::pair<double, ParamNode*>> results;
	while(heap.size() != 0) {
		results.push_back(heap.top());
		heap.pop();
	}
	return results;
}
void 
ParamTree::
getKNearestNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, int _n, int _depth, 
							   std::priority_queue<std::pair<double, ParamNode*>, std::vector<std::pair<double, ParamNode*>>, distComp>& _heap)
{
	if(_root == 0)
		return;
	double dist = getDistance(_root->getParam(), _pn);

	if(_heap.size() < _n)
		_heap.push(std::pair<double, ParamNode*>(dist, _root));	
	else {
		if(dist < _heap.top().first) {
			_heap.pop();
			_heap.push(std::pair<double, ParamNode*>(dist, _root));	
		}
	}

	int idx = _depth % mDim;
	Eigen::VectorXd projected = _pn;
	projected[idx] = _root->getParam()[idx];
	if(_root->getParam()[idx] <= _pn[idx]) {
		getKNearestNodesRecursive(_root->getRight(), _pn, _n, _depth+1, _heap);
		if(_heap.size() < _n || getDistance(projected, _pn) < _heap.top().first) {
			getKNearestNodesRecursive(_root->getLeft(), _pn, _n, _depth+1, _heap);
		}
	} else {
		getKNearestNodesRecursive(_root->getLeft(), _pn, _n, _depth+1, _heap);
		if(_heap.size() < _n || getDistance(projected, _pn) < _heap.top().first) {
			getKNearestNodesRecursive(_root->getRight(), _pn, _n, _depth+1, _heap);
		}
	}

}
std::vector<ParamNode*> 
ParamTree::
traverse()
{
	std::vector<ParamNode*> params;
	traverseRecursive(mRoot, params);
	return params;
}
void 
ParamTree::
traverseRecursive(ParamNode* _root, std::vector<ParamNode*>& _results)
{
	if (_root == 0)
		return;

	traverseRecursive(_root->getLeft(), _results);
	_results.push_back(_root);
	traverseRecursive(_root->getRight(), _results);

}
double 
ParamTree::
getDistance(Eigen::VectorXd _p0, Eigen::VectorXd _p1)
{
	double r = 0;
	for(int i = 0; i < mDim; i++) {
		r += pow((_p0(i) - _p1(i)) / mUnit(i), 2);
	}
	r /= mDim;
	return std::sqrt(r);
}
std::vector<ParamNode*> 
ParamTree::
traverseAndRebuild()
{
	std::vector<ParamNode*> params;
	traverseRecursive(mRoot, params);

	mRoot = buildRecursive(0, params);
	return params;
}
void 
ParamTree::
build(std::vector<ParamNode*> _params)
{
	mRoot = buildRecursive(0, _params);
}
ParamNode* 
ParamTree::
buildRecursive(int _depth, std::vector<ParamNode*> _params)
{
	if(_params.size() == 0)
		return 0;

	int idx = _depth % mDim;
	std::stable_sort(_params.begin(), _params.end(), [&idx] (ParamNode* i,ParamNode* j) { 
		return (i->getParam()[idx] < j->getParam()[idx]); 
	});

	int middle = _params.size()/2;
	for(int i = middle; i >= 0; i--) {
		if(i != middle && _params[middle]->getParam()[idx] == _params[i]->getParam()[idx]) 
			middle = i;
		else if(_params[middle]->getParam()[idx] > _params[i]->getParam()[idx])
			break;
	}
	ParamNode* root = _params[middle];
	std::vector<ParamNode*> leftHalf(_params.begin(), _params.begin() + middle);
	std::vector<ParamNode*> rightHalf(_params.begin() + (middle + 1), _params.end());

	ParamNode* left = buildRecursive(_depth + 1, leftHalf);
	ParamNode* right = buildRecursive(_depth + 1, rightHalf);

	root->setLeft(left);
	root->setRight(right);

	return root; 
}
void 
ParamTree::
clear()
{
	std::vector<ParamNode*> data = traverse();
	while(data.size() != 0) {
		ParamNode* p = data.back();
		data.pop_back();
		delete p;
	}
	mRoot = 0;
}
}
