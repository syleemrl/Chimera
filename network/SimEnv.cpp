#include "SimEnv.h"
#include <omp.h>
#include <ctime>
#include <iostream>
#include "dart/math/math.hpp"
#include "SkeletonBuilder.h"
#include "SimConfigParser.h"
SimEnv::
SimEnv(int _nslave, std::string _directory, 
	   std::string _simConfig, bool _pretrain, bool _noreg) :mNumSlave(_nslave)
{
	dart::math::seedRand();
	omp_set_num_threads(mNumSlave);

	std::string configPath;
	if(_pretrain)
		configPath = _directory + std::string("config.txt");
	else 
		configPath = std::string(SSC_DIR) + std::string("/data/config/") + _simConfig;
	SIM::SimConfig simConfig = SIM::SimConfigParser::parseSimConfig(configPath);

	mConfigPath.push_back(configPath);
	mConfigPath.push_back(simConfig.charPath);

    mReferenceManager = new SIM::ReferenceManager(simConfig, mNumSlave);
    
    mNoreg = _noreg;
    if(!simConfig.isAssembled && simConfig.numTimingFunc == 1)
    	mNoreg = true;

	for(int i = 0 ;i < mNumSlave; i++)
	{
		mSlaves.push_back(new SIM::Controller(mReferenceManager, simConfig, i));
		if(_pretrain) {
			mSlaves[i]->setPhaseDeltaWeight(1);
			mSlaves[i]->setFixWeight(false);
		}
	}

	mNumAction = mSlaves[0]->getNumAction();
	mNumState = mSlaves[0]->getNumState();
	mNumTimingFunc = mReferenceManager->getNumTimingFunc();
	
	mPath = _directory;

	Py_Initialize();
	np::initialize();
	try{
		if(!mNoreg) {
			p::object regression = p::import("regression");
			mRegressionNet = regression.attr("Regression")();
			mRegressionNet.attr("init_train")(_directory, mNumTimingFunc*2, mReferenceManager->getSkeletonDof(), _pretrain);
			
			if(!_pretrain)
				pretrainReference();
		}
	}
	catch (const p::error_already_set&)
	{
		PyErr_Print();
	}
}
void 
SimEnv::
saveTrainingData()
{
	std::cout << "save data to: " << mPath << "training_data.txt" << std::endl;
	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> data = mReferenceManager->getTrainingData();
	int dataSize = data.first.size();

	std::ofstream ofs(mPath + "training_data.txt");
	ofs << dataSize << " " << std::endl;
	for(int i = 0; i < dataSize; i++) {
		ofs << data.first[i].transpose() << std::endl;
		ofs << data.second[i].transpose() << std::endl;
	}

	ofs.close();

}
np::ndarray
SimEnv::
getCurrentFrameOnCycle()
{
	std::vector<Eigen::VectorXd> curFrames;
	for(int i =0 ; i < mNumSlave; i++) {
		curFrames.push_back(mSlaves[i]->getCurrentFrameOnCycle());
	}
	return SIM::toNumPyArray(curFrames);
}
void
SimEnv::
trainRegressionNetwork()
{
	if(mNoreg)
		return;

	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> data = mReferenceManager->getTrainingData();
	std::cout << "training data size: " << data.first.size() << std::endl;

	std::vector<Eigen::VectorXd> Xs;
	for(int i = 0; i < data.first.size(); i++) {
		Xs.push_back(mReferenceManager->toSinusoidalParam(data.first[i]));
	}
	
	// std::vector<Eigen::VectorXd> Xselected;
	// std::vector<Eigen::VectorXd> Yselected;
	// for(int i = 0; i < data.first.size(); i++) {
	// 	double density = mReferenceManager->getCurDensity(data.first[i]);
	// 	if(density > 1){
	// 		Xselected.push_back(Xs[i]);
	// 		Yselected.push_back(data.second[i]);
	// 	}

	// }
	// std::cout << "selected data size: " << Xselected.size() << std::endl;

	np::ndarray x_np = SIM::toNumPyArray(Xs);
	np::ndarray y_np = SIM::toNumPyArray(data.second);

	p::list l;
	l.append(x_np);
	l.append(y_np);

	mRegressionNet.attr("set_training_data")(l);
	p::object loss_p = mRegressionNet.attr("train")();
}
bool 
SimEnv::
optimizeReference() 
{
	if(mNoreg)
		return false;

	bool updated = mReferenceManager->optimizeReference();
	
	return updated;
}
void 
SimEnv::
pretrainReference()
{
	mReferenceManager->pretrainReference();

	std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> data = mReferenceManager->getTrainingData();

	std::vector<Eigen::VectorXd> Xs;
	for(int i = 0; i < data.first.size(); i++) {
		Xs.push_back(mReferenceManager->toSinusoidalParam(data.first[i]));
	}

	np::ndarray x_np = SIM::toNumPyArray(Xs);
	np::ndarray y_np = SIM::toNumPyArray(data.second);

	p::list l;
	l.append(x_np);
	l.append(y_np);

	mRegressionNet.attr("set_training_data")(l);
	p::object loss_p = mRegressionNet.attr("train")();
}
np::ndarray
SimEnv:: 
buildParamTree()
{
	std::vector<Eigen::VectorXd> curParamList = mReferenceManager->buildParamTree();
	return SIM::toNumPyArray(curParamList);
}
p::list 
SimEnv:: 
getDensity(np::ndarray _array, int _n)
{
	std::vector<Eigen::VectorXd> paramList = SIM::toEigenVector(_array, mNumTimingFunc, _n*mNumSlave);

	std::vector<std::vector<double>> densityList;
	for(int i = 0; i < mNumSlave; i++)
	{
		densityList.push_back(std::vector<double>());
	}

// #pragma omp parallel for
	for(int i = 0; i < mNumSlave; i++) {
		for(int j = 0; j < _n; j++) {
			double d = mReferenceManager->getDensity(paramList[i * _n + j]);
			densityList[i].push_back(d);
		}
	}

	p::list l;
	for(int i = 0; i < mNumSlave; i++) {
		for(int j = 0; j < _n; j++) {
			l.append(densityList[i][j]);
		}
	}	
	return l;
}
//For general properties
int
SimEnv::
getNumState()
{
	return mNumState;
}
int
SimEnv::
getNumAction()
{
	return mNumAction;
}
np::ndarray
SimEnv::
getMotionLength()
{
	Eigen::VectorXd l(mNumTimingFunc);
	for(int i = 0; i < mNumTimingFunc; i++) {
		l(i) = mReferenceManager->getMotionLength(i);
	}
	return SIM::toNumPyArray(l);
}
np::ndarray
SimEnv::
getCurrentFrame()
{
	std::vector<Eigen::VectorXd> curframe;
	for(int i =0 ; i < mNumSlave; i++) {
		curframe.push_back(mSlaves[i]->getCurrentFrame());
	}
	return SIM::toNumPyArray(curframe);
}
np::ndarray 
SimEnv::
getInitialTiming()
{
	std::vector<Eigen::VectorXd> initialTiming = mReferenceManager->getInitialTiming();
	return SIM::toNumPyArray(initialTiming);
}
p::list 
SimEnv::
getRewardLabels()
{
	p::list l;
	std::vector<std::string> sl = mSlaves[0]->getRewardLabels();
	for(int i =0 ; i < sl.size(); i++) l.append(sl[i]);
	return l;
}
//For each slave

void 
SimEnv::
reset(int _id, np::ndarray _start)
{
	mSlaves[_id]->resetTime(SIM::toEigenVector(_start, mNumTimingFunc));

	if(!mNoreg) {
		std::vector<Eigen::VectorXd> paramInput;

		Eigen::VectorXd param = mSlaves[_id]->getCurrentFrameOnCycle();
		Eigen::VectorXd nextFrame = mSlaves[_id]->getNextFrame();

		paramInput.push_back(mReferenceManager->toSinusoidalParam(param));
		param = nextFrame;
		paramInput.push_back(mReferenceManager->toSinusoidalParam(param));
		
		p::object out = mRegressionNet.attr("run")(SIM::toNumPyArray(paramInput));
		np::ndarray nout = np::from_object(out);
		Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, 2, mReferenceManager->getSkeletonDof());

		mSlaves[_id]->setTargetPosition(outputMatrix.row(0), outputMatrix.row(1));

	} else {
		Eigen::VectorXd zeroDisplcement = Eigen::VectorXd::Zero(mReferenceManager->getSkeletonDof());
		mSlaves[_id]->setTargetPosition(zeroDisplcement, zeroDisplcement);

	}

	

	mSlaves[_id]->reset();

}
p::tuple 
SimEnv::
getStepInfo(int _id)
{
	// is_terminal, nan_occur, nt_elapsed, t_elapsed
	bool t = mSlaves[_id]->isTerminalState();
	int tr = mSlaves[_id]->getTerminalReason();
	bool n = mSlaves[_id]->isNan();
	Eigen::VectorXd start = mSlaves[_id]->getStartFrame();
	Eigen::VectorXd cur = mSlaves[_id]->getCurrentFrame();
	double te = mSlaves[_id]->getTimeElapsed();

	Eigen::VectorXd elapsed = cur - start;
	return p::make_tuple(t, tr, n, SIM::toNumPyArray(start), SIM::toNumPyArray(cur), te);
}
np::ndarray
SimEnv::
getState(int _id)
{
	return SIM::toNumPyArray(mSlaves[_id]->getState());
}
np::ndarray
SimEnv::
getRewardVector(int _id)
{
	std::vector<double> ret = mSlaves[_id]->getRewardParts();

	return SIM::toNumPyArray(ret);
}
void
SimEnv::
stepAll()
{
#pragma omp parallel for
	for (int id = 0; id < mNumSlave; id++)
	{
		mSlaves[id]->stepTime();
	}

	if(!mNoreg) {
		std::vector<Eigen::VectorXd> paramInput;
		for (int id = 0; id < mNumSlave; id++) {

			Eigen::VectorXd param = mSlaves[id]->getCurrentFrameOnCycle();
			for(int j = 0; j < mNumTimingFunc; j++) {
				param(j) = std::fmod(param(j), mReferenceManager->getMotionLength(j));
			}
			paramInput.push_back(mReferenceManager->toSinusoidalParam(param));

			Eigen::VectorXd nextFrame = mSlaves[id]->getNextFrame();
			param = nextFrame;
			for(int j = 0; j < mNumTimingFunc; j++) {
				param(j) = std::fmod(param(j), mReferenceManager->getMotionLength(j));
			}
			paramInput.push_back(mReferenceManager->toSinusoidalParam(param));

		}


		p::object out = mRegressionNet.attr("run")(SIM::toNumPyArray(paramInput));
		np::ndarray nout = np::from_object(out);
		Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, mNumSlave*2, mReferenceManager->getSkeletonDof());

#pragma omp parallel for
		for (int id = 0; id < mNumSlave; id++)
		{
			mSlaves[id]->setTargetPosition(outputMatrix.row(2*id), outputMatrix.row(2*id+1));
		}

#pragma omp parallel for
		for (int id = 0; id < mNumSlave; id++)
		{
			mSlaves[id]->step();
		}

	} else {
		Eigen::VectorXd zeroDisplcement = Eigen::VectorXd::Zero(mReferenceManager->getSkeletonDof());

#pragma omp parallel for
		for (int id = 0; id < mNumSlave; id++)
		{
			mSlaves[id]->setTargetPosition(zeroDisplcement, zeroDisplcement);
			mSlaves[id]->step();

		}		
	}
}
np::ndarray
SimEnv::
getStateAll()
{
	Eigen::MatrixXd states(mNumSlave, mNumState);

	for (int id = 0; id < mNumSlave; id++)
	{
		states.row(id) = mSlaves[id]->getState().transpose();
	}
	return SIM::toNumPyArray(states);
}
void
SimEnv::
setActionAll(np::ndarray np_array)
{
	Eigen::MatrixXd action = SIM::toEigenMatrix(np_array, mNumSlave, mNumAction);

	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setAction(action.row(id).transpose());
	}
}
void 
SimEnv::
setWeightAll(double _weight) 
{
	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setPhaseDeltaWeight(_weight);
	}
}
void 
SimEnv::
setFixWeight(bool _fix)
{
	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setFixWeight(_fix);
	}
}
p::list 
SimEnv::
getConfigFilePath()
{
	p::list full;
	full.append(mConfigPath[0]);
	full.append(mConfigPath[1]);

	p::list name;
	name.append(SIM::getFileName(mConfigPath[0]));
	name.append(SIM::getFileName(mConfigPath[1]));

	p::list l;
	l.append(full);
	l.append(name);

	return l;
}
using namespace boost::python;

BOOST_PYTHON_MODULE(simEnv)
{
	Py_Initialize();
	np::initialize();

	class_<SimEnv>("Env", init<int, std::string, std::string, bool, bool>())
		.def("getNumState",&SimEnv::getNumState)
		.def("getNumAction",&SimEnv::getNumAction)
		.def("getMotionLength",&SimEnv::getMotionLength)
		.def("getRewardLabels",&SimEnv::getRewardLabels)
		.def("reset",&SimEnv::reset)
		.def("getState",&SimEnv::getState)
		.def("getCurrentFrame",&SimEnv::getCurrentFrame)
		.def("getInitialTiming",&SimEnv::getInitialTiming)
		.def("getStepInfo",&SimEnv::getStepInfo)
		.def("getRewardVector",&SimEnv::getRewardVector)
		.def("stepAll",&SimEnv::stepAll)
		.def("getStateAll",&SimEnv::getStateAll)
		.def("setActionAll",&SimEnv::setActionAll)
		.def("setWeightAll",&SimEnv::setWeightAll)
		.def("getCurrentFrameOnCycle",&SimEnv::getCurrentFrameOnCycle)
		.def("trainRegressionNetwork",&SimEnv::trainRegressionNetwork)
		.def("optimizeReference",&SimEnv::optimizeReference)
		.def("pretrainReference",&SimEnv::pretrainReference)
		.def("buildParamTree",&SimEnv::buildParamTree)
		.def("getDensity",&SimEnv::getDensity)
		.def("setFixWeight",&SimEnv::setFixWeight)
		.def("getConfigFilePath",&SimEnv::getConfigFilePath);
}