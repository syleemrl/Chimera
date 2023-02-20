#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "SingleMotionWidget.h"
#include "SkeletonBuilder.h"
#include "SimConfigParser.h"
SingleMotionWidget::
SingleMotionWidget()
  :mCamera(new Camera(1600, 900)), mCurFrame(0), mTotalFrame(0), mPlay(false), mTrackCamera(false), 
   mDrawRef(true), mSimConfigExists(false)
{
	startTimer(30);
}
SingleMotionWidget::
SingleMotionWidget(std::vector<RenderData> _renderData)
  :SingleMotionWidget()
{
	mRenderData = _renderData;

    Py_Initialize();
    np::initialize();

	setRenderData(0);
	setFocusPolicy( Qt::StrongFocus );
}
void
SingleMotionWidget::
setNetworkSetting(std::string _directory) {
 	mRunRegression = true;

    try {
		p::object sys_module = p::import("sys");
		p::str module_dir = (std::string(SSC_DIR)+"/network").c_str();
		sys_module.attr("path").attr("insert")(1, module_dir);
    	mController = new SIM::Controller(mReferenceManager, mSimConfig, 0, true);
    	
    	p::object ppo_main = p::import("ppo");
		mPPO = ppo_main.attr("PPO")();	
		std::string ppoPath = std::string(SSC_DIR)+ std::string("/network/output/") + _directory + std::string("/network-0");
		mPPO.attr("init_run")(ppoPath,
							 mController->getNumState(), 
							 mController->getNumAction(),
							 mSimConfig.numTimingFunc);    	

		std::string vsiPath = std::string(SSC_DIR)+ std::string("/network/output/") + _directory + std::string("/vsi-0");
		mStartDistribution = RenderConfigParser::parseStartDistribution(vsiPath, mSimConfig.numTimingFunc);

   		std::string regPath = std::string(SSC_DIR)+ std::string("/network/output/") + _directory;

		p::object regression = p::import("regression");
		mRegressionNet = regression.attr("Regression")();
		mRegressionNet.attr("init_run")(regPath, 2*nTimingFuncs, mReferenceManager->getSkeletonDof());
    } catch (const p::error_already_set&) {
    	mRunRegression = false;
        PyErr_Print();
    }    
}
void
SingleMotionWidget::
runPPO(bool _throwBall) {
	if(mStartDistribution.first)
		mController->resetTime(mStartDistribution.second[0]);
	else
		mController->resetTime();
	if(mRunRegression) {
		std::vector<Eigen::VectorXd> paramInput;

		Eigen::VectorXd param = mController->getCurrentFrameOnCycle();
		paramInput.push_back(mReferenceManager->toSinusoidalParam(param));

		param = mController->getNextFrame();
		paramInput.push_back(mReferenceManager->toSinusoidalParam(param));

		p::object out = mRegressionNet.attr("run")(SIM::toNumPyArray(paramInput));
		np::ndarray nout = np::from_object(out);
		Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, 2, mReferenceManager->getSkeletonDof());

		mController->setTargetPosition(outputMatrix.row(0), outputMatrix.row(1));
	} else {
		Eigen::VectorXd zeroDisplacement = Eigen::VectorXd::Zero(mSkelPPO->getNumDofs());
		mController->setTargetPosition(zeroDisplacement, zeroDisplacement);
	}


	mController->reset();

	int count = 0;
	while(!mController->isTerminalState()) {
		Eigen::VectorXd state = mController->getState();

		p::object a = mPPO.attr("run")(SIM::toNumPyArray(state));
		np::ndarray na = np::from_object(a);
		Eigen::VectorXd action = SIM::toEigenVector(na, mController->getNumAction());
		mController->setAction(action);

		mController->stepTime();

		if(mRunRegression) {
			std::vector<Eigen::VectorXd> paramInput;

			Eigen::VectorXd param = mController->getCurrentFrameOnCycle();
			paramInput.push_back(mReferenceManager->toSinusoidalParam(param));

			param = mController->getNextFrame();
			paramInput.push_back(mReferenceManager->toSinusoidalParam(param));
			p::object out = mRegressionNet.attr("run")(SIM::toNumPyArray(paramInput));
			np::ndarray nout = np::from_object(out);
			Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, 2, mReferenceManager->getSkeletonDof());

			mController->setTargetPosition(outputMatrix.row(0), outputMatrix.row(1));
		} else {
			Eigen::VectorXd zeroDisplacement = Eigen::VectorXd::Zero(mSkelPPO->getNumDofs());
			mController->setTargetPosition(zeroDisplacement, zeroDisplacement);				
		}
	
		mController->step();
		count += 1;
	}

	mTiming.clear();
    mMotionPPO.clear();
    mMotionRef.clear();

	for(int i = 0; i < count; i++) {
		Eigen::VectorXd phase = mController->getPhase(i);
		mTiming.push_back(phase);
		
		Eigen::VectorXd position = mController->getTargetPosition(i);
		mMotionRef.push_back(position);

		position = mController->getPosition(i);
		position(3) += 1.5;
		mMotionPPO.push_back(position);

	}

	mTotalFrame = mMotionPPO.size() - 1;
	mCurFrame = 0;
}
void
SingleMotionWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
SingleMotionWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
SingleMotionWidget::
setFrame(int n)
{
    mSkelRef->setPositions(mMotionRef[n]);
    if(mRunSimulation) {
    	mSkelPPO->setPositions(mMotionPPO[n]);
    }
    if(mPhaseWidget.size() != 0) {
	    for(int i = 0; i < nTimingFuncs; i++) {
	    	mPhaseWidget[i]->setValue((int) std::floor(mTiming[n](i)*1000));
		    QLabel *label = mPhaseWidget[i]->findChild<QLabel *>();
			label->setText(QString::number(mTiming[n](i)));
	    }
    }
}
void
SingleMotionWidget::
drawSkeletons()
{
	if(mRunSimulation) {
		GUI::drawSkeleton(mSkelPPO, 0);
	}

	if(mDrawRef) {
		GUI::drawSkeleton(mSkelRef, 0);
	}
}	
void
SingleMotionWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelRef->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
SingleMotionWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);
	
	if(mTrackCamera){
		Eigen::Vector3d com = mSkelRef->getRootBodyNode()->getCOM();
		if(mRunSimulation) {
			com = mSkelPPO->getRootBodyNode()->getCOM();
		}
		com[1] = 0.8;
		mCamera->setCenter(com);
	}
	mCamera->apply();

	drawGround();
	drawSkeletons();
	
	GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());

}
void
SingleMotionWidget::
initLights()
{

	float ambient[]           	 = {0.4, 0.4, 0.4, 1.0};
	float diffuse[]             = {0.4, 0.4, 0.4, 1.0};
	float frontShininess[] = {60.0};
	float frontSpecular[]  = {0.2, 0.2,  0.2,  1.0};
	float frontDiffuse[]   = {0.2, 0.2, 0.2, 1.0};
	float lmodelAmbient[]      = {0.2, 0.2,  0.2,  1.0};
	float lmodelTwoside[]      = {GL_TRUE};

	GLfloat position[] = {0.0, 1.0, 1.0, 0.0};
	GLfloat position1[] = {0.0, 1.0, -1.0, 0.0};

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodelAmbient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodelTwoside);

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position1);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, frontShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  frontSpecular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   frontDiffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glEnable(GL_FOG);
	GLfloat fogColor[] = {200.0/256.0, 200.0/256.0, 200.0/256.0, 1};
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_DENSITY, 0.05);
	glFogf(GL_FOG_START, 20.0);
	glFogf(GL_FOG_END, 40.0);
}
void
SingleMotionWidget::
timerEvent(QTimerEvent* _event)
{
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurFrame);
	update();

}
void
SingleMotionWidget::
keyPressEvent(QKeyEvent* _event)
{
	if(_event->key() == Qt::Key_Escape){
		exit(0);
	}
	if(_event->key() == Qt::Key_Space){
		mPlay = !mPlay;
		if(mPlay)
			std::cout << "Play." << std::endl;
		else 
			std::cout << "Pause." << std::endl;
	}
	if(_event->key() == Qt::Key_T){
		mTrackCamera = !mTrackCamera;
		std::cout << "track camera: " << mTrackCamera << std::endl;
	}
	if(_event->key() == Qt::Key_1){
		mDrawRef = !mDrawRef;
	}
}
void
SingleMotionWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
SingleMotionWidget::
mouseMoveEvent(QMouseEvent* _event)
{
	if(!mIsDrag)
	return;

	if (mButton == Qt::MidButton)
		mCamera->translate(_event->x(), _event->y(), mPrevX, mPrevY);
	else if(mButton == Qt::LeftButton)
		mCamera->rotate(_event->x(), _event->y(), mPrevX, mPrevY);

	mPrevX = _event->x();
	mPrevY = _event->y();
	update();
}
void
SingleMotionWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
SingleMotionWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
SingleMotionWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
SingleMotionWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
SingleMotionWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
SingleMotionWidget::
togglePlay() {
	mPlay = !mPlay;
}
void 
SingleMotionWidget::
setRenderData(int _idx) {

	RenderData data = mRenderData[_idx];
	
	mSimConfigExists = false;
	if(data.network == "")
		mRunSimulation = false;
	else 
		mRunSimulation = true;

	mCurFrame = 0;
	mTotalFrame = 0;

	int terminalIteration = 10;

	//bvh
	if(data.CMpair.size() != 0) {
		std::string skelPath = std::string(SSC_DIR) + std::string("/data/character/") + (data.CMpair[0]).first;
		std::string motionPath = std::string(SSC_DIR) + std::string("/data/motion/") + (data.CMpair[0]).second;
		if(!boost::filesystem::exists(skelPath)) {
			SIM::SkeletonBuilder::generateNewSkeleton(motionPath, skelPath);
		}
	    mReferenceManager = new SIM::ReferenceManager(SIM::SkeletonBuilder::buildFromFile(skelPath));

		mReferenceManager->loadMotionFromBVH(motionPath);
		mReferenceManager->setTimeStep();

	    mSkelPPO = mReferenceManager->getSkeletonCopy();
	    mSkelRef = mReferenceManager->getSkeletonCopy();
	
	//simconfig
	} else {
		std::string configPath;
		if(data.network != "")
			configPath = std::string(SSC_DIR) + std::string("/network/output/") + data.network + std::string("/config.txt");
		else
			configPath = std::string(SSC_DIR) + std::string("/data/config/") + data.simConfig;

		mSimConfig = SIM::SimConfigParser::parseSimConfig(configPath);
		SIM::SimConfigParser::printSimConfig(mSimConfig);

		terminalIteration = mSimConfig.tc.terminalIteration;
    
	    mReferenceManager = new SIM::ReferenceManager(mSimConfig);

		mSkelPPO = mReferenceManager->getSkeletonCopy();
		mSkelRef = mReferenceManager->getSkeletonCopy();		

		mSimConfigExists = true;
	}
	nTimingFuncs = mReferenceManager->getNumTimingFunc();

	GUI::setSkeletonColor(mSkelRef, Eigen::Vector4d(255./255., 99./255., 71./255., 1.0));
	GUI::setSkeletonColor(mSkelPPO, Eigen::Vector4d(235./255., 235./255., 235./255., 1.0));

	// set phase widget
	if(mPhaseWidget.size() != 0) {
		int nfuncs = nTimingFuncs;
	    for(int i = 0; i < nfuncs; i++) {
	        mPhaseWidget[i]->setMaximum(1000);
	        mPhaseWidget[i]->setValue(0);
	        mPhaseWidget[i]->setStyleSheet("background-color:white;");
  
	        QLabel *label = mPhaseWidget[i]->findChild<QLabel *>();
	        label->setStyleSheet("color:black; background-color: rgba(0,0,0,0%);");
			label->setText(QString::number(0));
	    }  
	    for(int i = nfuncs; i < mPhaseWidget.size(); i++) {
	        mPhaseWidget[i]->setValue(0);
	        mPhaseWidget[i]->setStyleSheet("background-color:#EFEAE8;"); 
	        QLabel *label = mPhaseWidget[i]->findChild<QLabel *>();
			label->setText("");
	    }
	}
	// generate motion

	if(mRunSimulation) {
		setNetworkSetting(data.network);
		runPPO();

	} else {
		Eigen::VectorXd phase(nTimingFuncs);
		phase.setZero();

		bool endFlag = false;
			
		mMotionRef.clear();
		mTiming.clear();


		Eigen::VectorXd phaseNormalized = phase;
		Eigen::VectorXd tPrev = phase;

		Eigen::Vector6d rootPrev = mReferenceManager->getFrame(phase).segment<6>(0);

		while(1) {

			for(int i = 0; i < nTimingFuncs; i++) {		
				phaseNormalized(i) = std::fmod(phase(i) / mReferenceManager->getMotionLength(i), 1.0);
			}

			mTiming.push_back(phaseNormalized);
				
			Eigen::VectorXd phaseOnCycle = phase;
			Eigen::VectorXd phasePrevOnCycle = tPrev;
			for(int i = 0; i < nTimingFuncs; i++) {		
				phaseOnCycle(i) = std::fmod(phaseOnCycle(i), mReferenceManager->getMotionLength(i));
				phasePrevOnCycle(i) = std::fmod(phasePrevOnCycle(i), mReferenceManager->getMotionLength(i));
			} 
			if(mSimConfigExists) {
				Eigen::VectorXd p;
			   	if(mMotionRef.size() == 0) 
			   		p = mReferenceManager->getPosSynthesized(phaseOnCycle);
			   	else 
			   		p = mReferenceManager->getPosSynthesized(phaseOnCycle, phasePrevOnCycle, rootPrev);

			   	mMotionRef.push_back(p);	
			   	rootPrev = p.segment<6>(0);

			} else {
				Eigen::VectorXd p;
			   	if(mMotionRef.size() == 0) 
			   		p = mReferenceManager->getBaseFrame(phaseOnCycle(0));
			   	else
			   		p = mReferenceManager->getBaseFrame(phaseOnCycle(0), phasePrevOnCycle(0), rootPrev);
			   
			   	mMotionRef.push_back(p);	
			   	rootPrev = p.segment<6>(0);
			}
		
			tPrev = phase;
			phase += mReferenceManager->getTimeStep(phase);
			for(int i = 0; i < nTimingFuncs; i++) {
				if(phase(i) >= mReferenceManager->getMotionLength(i) * terminalIteration) {
					endFlag = true;
					break;
				}
			}
			if(endFlag)
				break;
		}

		mTotalFrame = mMotionRef.size() - 1;
	}
}
