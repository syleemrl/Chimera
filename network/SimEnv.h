#ifndef __SIM_ENV_H__
#define __SIM_ENV_H__
#include "Controller.h"
#include "ReferenceManager.h"
#include "Functions.h"
#include <vector>
#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;
class SimEnv
{
public:
	
	SimEnv(int _nslave, std::string _directory, 
		   std::string _simConfig,
		   bool _pretrain, bool _noreg);

	//For general properties
	int getNumState();
	int getNumAction();
	np::ndarray getMotionLength();
	p::list getRewardLabels();
	np::ndarray getCurrentFrame();

	//For each slave
	void reset(int _id, np::ndarray _start);
	p::tuple getStepInfo(int _id);
	np::ndarray getState(int _id);
	np::ndarray getRewardVector(int _id);

	//For all slaves
	void stepAll();
	void setActionAll(np::ndarray _array);
	void setWeightAll(double _weight);
	np::ndarray getStateAll();
	void setFixWeight(bool _fix);

	np::ndarray getInitialTiming();
	np::ndarray getCurrentFrameOnCycle();

	void trainRegressionNetwork();
	bool optimizeReference();
	void pretrainReference();
	
	np::ndarray buildParamTree();
	p::list getDensity(np::ndarray _array, int _n);

	void saveTrainingData();
	p::list getConfigFilePath();
private:
	std::vector<SIM::Controller*> mSlaves;
	SIM::ReferenceManager* mReferenceManager;
	p::object mRegressionNet;

	int mNumSlave;
	int mNumAction;
	int mNumState;
	int mNumBase;
	int mNumTimingFunc;

	bool mNoreg;

	std::string mPath;
	std::vector<std::string> mConfigPath;

};


#endif