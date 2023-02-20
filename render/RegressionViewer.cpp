#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "RegressionViewer.h"
#include "SkeletonBuilder.h"
#include "SimConfigParser.h"
RegressionViewer::
RegressionViewer()
  :mCamera(new Camera(1600, 900)), mPlay(false), mDrawNet(true), mDrawIK(true)
{
	startTimer(30);
}
RegressionViewer::
RegressionViewer(std::vector<RenderData> _renderData)
  :RegressionViewer()
{
	mRenderData = _renderData;

    Py_Initialize();
    np::initialize();

	setRenderData(0);
	setFocusPolicy( Qt::StrongFocus );
}
void 
RegressionViewer::
setTiming(const int& _value) {
	auto slider = qobject_cast<QSlider*>(sender());
    auto i = slider->property("i").toInt();
    mInputPhase(i) =  _value * 1 / 100.0 * mReferenceManager->getMotionLength(i);
    
}
void
RegressionViewer::
setNetworkSetting(std::string _network) {
    try {
   		std::string path = std::string(SSC_DIR)+ std::string("/network/output/") + _network;
		p::object regression = p::import("regression");
		mRegressionNet = regression.attr("Regression")();
		mRegressionNet.attr("init_run")(path, 2*nTimingFuncs, mReferenceManager->getSkeletonDof());
	
    } catch (const p::error_already_set&) {
        PyErr_Print();
    }    
}
void
RegressionViewer::
runRegression() {

	mCurFrame = 0;
	mTotalFrame = 0;

	mTiming.clear();
	mMotionIK.clear();
	mMotionNet.clear();

	std::vector<Eigen::VectorXd> phaseTrajectory;
	std::vector<Eigen::VectorXd> paramInput;
	Eigen::VectorXd param = mInputPhase;

	for(int i = 0; i < mOverhead; i++) {
		phaseTrajectory.push_back(param);
		paramInput.push_back(mReferenceManager->toSinusoidalParam(param));

		param += mReferenceManager->getTimeStep(param);
	}

	p::object out = mRegressionNet.attr("run")(SIM::toNumPyArray(paramInput));
	np::ndarray nout = np::from_object(out);
	Eigen::MatrixXd outputMatrix = SIM::toEigenMatrix(nout, mOverhead, mReferenceManager->getSkeletonDof());

	for(int i = 0; i < mOverhead; i++) {
		Eigen::VectorXd pIK = mReferenceManager->getPosSynthesized(phaseTrajectory[i]);
		Eigen::VectorXd p = mReferenceManager->getPositionWithDisplacement(phaseTrajectory[i], outputMatrix.row(i)).second;
		mTiming.push_back(phaseTrajectory[i]);
		mMotionIK.push_back(pIK);
		mMotionNet.push_back(p);
	}

	mTotalFrame = mOverhead - 1;
}
void
RegressionViewer::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
RegressionViewer::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
RegressionViewer::
setFrame(int n)
{
    mSkelIK->setPositions(mMotionIK[n]);
    mSkelNet->setPositions(mMotionNet[n]);
}
void
RegressionViewer::
drawSkeletons()
{
	if(mDrawIK)
		GUI::drawSkeleton(mSkelIK, 0);
	if(mDrawNet)
		GUI::drawSkeleton(mSkelNet, 0);

}	
void
RegressionViewer::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelNet->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
RegressionViewer::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);
	
	mCamera->apply();

	drawGround();
	drawSkeletons();
	
	GUI::drawStringOnScreen(0.8, 0.9, SIM::vec2Str(mTiming[mCurFrame]), true, Eigen::Vector3d::Zero());

}
void
RegressionViewer::
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
RegressionViewer::
timerEvent(QTimerEvent* _event)
{
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurFrame);
	update();

}
void
RegressionViewer::
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
	if(_event->key() == Qt::Key_0) {
		mDrawIK = !mDrawIK;
		std::cout << "Draw Optimization output: " << mDrawIK << std::endl;
	}
	if(_event->key() == Qt::Key_1) {
		mDrawNet = !mDrawNet;
		std::cout << "Draw Network output: " << mDrawNet << std::endl;
	}
}
void
RegressionViewer::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
RegressionViewer::
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
RegressionViewer::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
RegressionViewer::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
RegressionViewer::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
RegressionViewer::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
RegressionViewer::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
RegressionViewer::
togglePlay() {
	mPlay = !mPlay;
}
void 
RegressionViewer::
setRenderData(int _idx) {

	RenderData data = mRenderData[_idx];

	std::string configPath = std::string(SSC_DIR) + std::string("/network/output/") + data.network + std::string("/config.txt");
	mSimConfig = SIM::SimConfigParser::parseSimConfig(configPath);
	SIM::SimConfigParser::printSimConfig(mSimConfig);

	mReferenceManager = new SIM::ReferenceManager(mSimConfig);

	mSkelIK = mReferenceManager->getSkeletonCopy();
	mSkelNet = mReferenceManager->getSkeletonCopy();

	nTimingFuncs = mReferenceManager->getNumTimingFunc();

	GUI::setSkeletonColor(mSkelIK, Eigen::Vector4d(255./255., 142./255., 124./255., 1.0));
	GUI::setSkeletonColor(mSkelNet, Eigen::Vector4d(255./255., 190./255., 183./255., 1.0));

	mInputPhase.resize(nTimingFuncs);
	mInputPhase.setZero();

	setNetworkSetting(data.network);
	runRegression();
}