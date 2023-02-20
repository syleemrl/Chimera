#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <QCheckBox>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "AssemblerWidget.h"
#include "SkeletonBuilder.h"
#include "SimConfigParser.h"
AssemblerWidget::
AssemblerWidget()
  :mCamera(new Camera(1600, 900)), mCurFrame(0), mTotalFrame(0), mTrackCamera(false),  
  mPlayMode(false), mEditMode(false)
{
	startTimer(30);
}
AssemblerWidget::
AssemblerWidget(std::vector<RenderData> _renderData)
  :AssemblerWidget()
{
	mRenderData = _renderData;

    Py_Initialize();
    np::initialize();

	setRenderData(0);
	setFocusPolicy( Qt::StrongFocus );
}
void
AssemblerWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
AssemblerWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
AssemblerWidget::
setFrame(int n)
{
	if(!mEditMode) {
		mSkelRef->setPositions(mMotionRef[n]);
	  mMotionRefPreview = mMotionRef[n];

		for(int i = 0; i < nTimingFuncs; i++) {
		    mSkelRefBase[i]->setPositions(mMotionRefBase[i][n]);		   
		    mMotionRefBasePreview[i] = mMotionRefBase[i][n];
		}

	  if(mPhaseWidgets.size() != 0) {
		  for(int i = 0; i < nTimingFuncs; i++) {
		    mPhaseWidgets[i]->setValue((int) std::floor(mTiming[n](i)*1000));
			  QLabel *label = mPhaseWidgets[i]->findChild<QLabel *>();
				label->setText(QString::number(mTiming[n](i)));
		  }
	  }
		for(int i = 0; i < nTimingFuncs; i++) {
	   	 	mInputPhase(i) = mTiming[n](i) * mReferenceManager->getMotionLength(i);
		}
	} else {
		mSkelRef->setPositions(mMotionRefPreview);
		for(int i = 0; i < nTimingFuncs; i++) {
		    mSkelRefBase[i]->setPositions(mMotionRefBasePreview[i]);
		}
	}
}
void
AssemblerWidget::
drawSkeletons()
{
	GUI::drawSkeleton(mSkelRef, 0);

	for(int i = 0; i < nTimingFuncs; i++) {
		GUI::drawSkeleton(mSkelRefBase[i], 0);	
	}

}	
void
AssemblerWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelRef->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
AssemblerWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);
	
	if(mTrackCamera){
		Eigen::Vector3d com = mSkelRef->getRootBodyNode()->getCOM();
		com[1] = 0.8;
		mCamera->setCenter(com);
	}
	mCamera->apply();

	drawGround();
	drawSkeletons();
	
	GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());

}
void
AssemblerWidget::
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
AssemblerWidget::
timerEvent(QTimerEvent* _event)
{
	if(mPlayMode && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurFrame);
	update();
}
void
AssemblerWidget::
keyPressEvent(QKeyEvent* _event)
{
	if(_event->key() == Qt::Key_Escape){
		exit(0);
	}
	if(_event->key() == Qt::Key_Space){
		mPlayMode = !mPlayMode;
		if(mPlayMode)
			std::cout << "Play." << std::endl;
		else 
			std::cout << "Pause." << std::endl;
	}
	if(_event->key() == Qt::Key_T){
		mTrackCamera = !mTrackCamera;
		std::cout << "track camera: " << mTrackCamera << std::endl;
	}
}
void
AssemblerWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
AssemblerWidget::
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
AssemblerWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
AssemblerWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
AssemblerWidget::
nextFrame()
{ 
	mEditMode = false;
	if(!mPlayMode) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
AssemblerWidget::
prevFrame()
{
	mEditMode = false;
	if(!mPlayMode && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
AssemblerWidget::
reset()
{
	mEditMode = false;
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
AssemblerWidget::
togglePlay() {
	mEditMode = false;
	mPlayMode = !mPlayMode;
}
void 
AssemblerWidget::
setRenderData(int _idx) {
	mEditMode = false;

	RenderData data = mRenderData[_idx];

	mCurFrame = 0;
	mTotalFrame = 0;

	std::string configPath = std::string(SSC_DIR) + std::string("/data/config/") + data.simConfig;

	mSimConfig = SIM::SimConfigParser::parseSimConfig(configPath);
	SIM::SimConfigParser::printSimConfig(mSimConfig);

	mReferenceManager = new SIM::ReferenceManager(mSimConfig);
	nTimingFuncs = mReferenceManager->getNumTimingFunc();

	mSkelRef = mReferenceManager->getSkeletonCopy();
		
	mInputPhase.resize(nTimingFuncs);
	
	mPhaseActivated.clear();
	for(int i = 0; i < nTimingFuncs; i++) {
		mPhaseActivated.push_back(false);
	}
	
	mInputStyle.resize(nTimingFuncs);
	mInputStyle.setZero();
	
	mStyleActivated.clear();
	for(int i = 0; i < nTimingFuncs; i++) {
		mStyleActivated.push_back(false);
	}
	mSpeedActivated = false;

	mSkelRefBase.clear();
	for(int i =0 ; i < nTimingFuncs; i++) {
		int idx = mReferenceManager->getPhaseBaseMapping(i);
		mSkelRefBase.push_back(mReferenceManager->getBaseSkelCopy(idx));
		GUI::setSkeletonColor(mSkelRefBase[i], Eigen::Vector4d(235./255., 235./255., 235./255., 1.0));		
	}

	GUI::setSkeletonColor(mSkelRef, Eigen::Vector4d(255./255., 99./255., 71./255., 1.0));
	
	// set phase widget
	if(mPhaseWidgets.size() != 0) {
		int nfuncs = nTimingFuncs;
	    for(int i = 0; i < nfuncs; i++) {
	        mPhaseWidgets[i]->setMaximum(1000);
	        mPhaseWidgets[i]->setValue(0);
	        mPhaseWidgets[i]->setStyleSheet("background-color:white;");
 
	        QLabel *label = mPhaseWidgets[i]->findChild<QLabel *>();
	        label->setStyleSheet("color:black; background-color: rgba(0,0,0,0%);");
			label->setText(QString::number(0));
	    }  
	    for(int i = nfuncs; i < mPhaseWidgets.size(); i++) {
	        mPhaseWidgets[i]->setValue(0);
	        mPhaseWidgets[i]->setStyleSheet("background-color:#EFEAE8;"); 

	        QLabel *label = mPhaseWidgets[i]->findChild<QLabel *>();
			label->setText("");
	    }
	}

	Eigen::VectorXd phase(nTimingFuncs);
	phase.setZero();
		
	mMotionRef.clear();
	mTiming.clear();

	mMotionRefBase.clear();
	for(int i = 0 ; i < nTimingFuncs; i++) {
		mMotionRefBase.push_back(std::vector<Eigen::VectorXd>());
		mMotionRefBasePreview.push_back(Eigen::VectorXd::Zero(1));
	}
		

	Eigen::VectorXd phaseNormalized = phase;
	bool endFlag = false;

	while(1) {
		for(int i = 0; i < nTimingFuncs; i++) {		
			phaseNormalized(i) = std::fmod(phase(i) / mReferenceManager->getMotionLength(i), 1.0);

		}

		mTiming.push_back(phaseNormalized);

		Eigen::VectorXd p = mReferenceManager->getPosSynthesized(phase);
		mMotionRef.push_back(p);	

 		for(int i = 0; i < nTimingFuncs; i++) {			
			Eigen::VectorXd pb = mReferenceManager->getBaseFrame(phase(i), i);
			pb(3) += (i+1) * 2 + 1;		
			mMotionRefBase[i].push_back(pb);
		}
		  
		phase += mReferenceManager->getTimeStep(phase);
		for(int i = 0; i < nTimingFuncs; i++) {
			if(phase(i) >= mReferenceManager->getMotionLength(i)) {
				endFlag = true;
				break;
			}
		}
		if(endFlag)
			break;
	}

	mTotalFrame = mMotionRef.size() - 1;

}
void 
AssemblerWidget::
setStyle(const int& _value) {
	mEditMode = true;

	auto slider = qobject_cast<QSlider*>(sender());
  auto i = slider->property("i").toInt();
  mInputStyle(i) = _value / 10.0;
  setPreview(); 
}
void 
AssemblerWidget::
setSpeed(const int& _value) {
	mInputSpeed = 0.5 + _value / 10.0;
}
void 
AssemblerWidget::
phaseActivated() {
	mEditMode = true;

	auto box = qobject_cast<QCheckBox*>(sender());
	auto i = box->property("i").toInt();
	mPhaseActivated[i] = box->isChecked();
}
void 
AssemblerWidget::
styleActivated() {
	mEditMode = true;

	auto box = qobject_cast<QCheckBox*>(sender());
	auto i = box->property("i").toInt();
	mStyleActivated[i] = box->isChecked();
}
void 
AssemblerWidget::
speedActivated() {
	auto box = qobject_cast<QCheckBox*>(sender());
	mSpeedActivated = box->isChecked();

}
void 
AssemblerWidget::
setPhase(const int& _value) {
	mEditMode = true;

	auto slider = qobject_cast<QSlider*>(sender());
  auto i = slider->property("i").toInt();
  mInputPhase(i) = _value / 100.0 * mReferenceManager->getMotionLength(i);
	mMotionRefBasePreview[i] = mReferenceManager->getBaseFrame(mInputPhase(i), i);
	mMotionRefBasePreview[i](3) += (i+1) * 2 + 1;	

	setPreview(); 
}
void 
AssemblerWidget::
setPreview() {
	Eigen::VectorXd activatedPhase(nTimingFuncs);
	for(int i = 0; i < nTimingFuncs; i++) {
		if(mPhaseActivated[i])
			activatedPhase[i] = mInputPhase[i];
		else
			activatedPhase[i] = mTiming[mCurFrame](i) * mReferenceManager->getMotionLength(i);
	}
	Eigen::VectorXd activatedStyle = mReferenceManager->getStyle(activatedPhase);
	for(int i = 0; i < nTimingFuncs; i++) {
		if(mStyleActivated[i])
			activatedStyle[i] = mInputStyle[i];
	}
	mMotionRefPreview = mReferenceManager->getPosSynthesized(activatedPhase, activatedStyle);
}
void 
AssemblerWidget::
setSliderCurrent() {
	for(int i = 0; i < nTimingFuncs; i++) {
		mTimelines[i]->setValue((int)floor(mTiming[mCurFrame](i) * 100));
	}
}  
void
AssemblerWidget::
editMotion() {
	Eigen::VectorXd phase(nTimingFuncs);
	mTiming.clear();
	phase.setZero();
			
	mMotionRef.clear();
	for(int i = 0 ; i < nTimingFuncs; i++) {
		mMotionRefBase[i].clear();
	}

	Eigen::VectorXd phaseNormalized = phase;	
	bool endFlag = false;

	while(1) {
		for(int i = 0; i < nTimingFuncs; i++) {		
			phaseNormalized(i) = std::fmod(phase(i) / mReferenceManager->getMotionLength(i), 1.0);
		}

		mTiming.push_back(phaseNormalized);
		Eigen::VectorXd p = mReferenceManager->getPosSynthesized(phase);
		mMotionRef.push_back(p);	


		for(int i = 0; i < nTimingFuncs; i++) {			
			Eigen::VectorXd pb = mReferenceManager->getBaseFrame(phase(i), i);
			pb(3) += (i+1) * 2 + 1;		
			mMotionRefBase[i].push_back(pb);
		}
		phase += mReferenceManager->getTimeStep(phase);

		for(int i = 0; i < nTimingFuncs; i++) {
			if(phase(i) >= mReferenceManager->getMotionLength(i)) {
				endFlag = true;
				break;
			}
		}
		if(endFlag)
			break;
	}
	mTotalFrame = mMotionRef.size() - 1;

}
std::string 
AssemblerWidget::
addConstraint() {
	std::string str = "";
	for(int i = 0; i < mPhaseActivated.size(); i++) {
		if(mPhaseActivated[i]) {
			if(str != "")
				str += ", ";
			str += "phase " + std::to_string(i) + ": " + std::to_string(mInputPhase[i]);
		}
	}
	for(int i = 0; i < mStyleActivated.size(); i++) {
		if(mStyleActivated[i]) {
			if(str != "")
				str += ", ";
			str += "style " + std::to_string(i) + ": " + std::to_string(mInputStyle[i]);
		}
	}

	if(mSpeedActivated) {
		if(str != "")
			str += ", ";
		str += "speed: " + std::to_string(mInputSpeed);
	}
	mReferenceManager->addConstraint(mInputPhase, mInputStyle, mInputSpeed,
									 mPhaseActivated, mStyleActivated, mSpeedActivated);
	editMotion();

	return str;
}
void 
AssemblerWidget::
printCurrentSettings() {
	std::vector<Eigen::VectorXd> constraintSync = mReferenceManager->getConstraintSync();
	std::vector<std::tuple<int, double, double>> constraintStyle = mReferenceManager->getConstraintStyle();

	std::cout << "constraints: " << std::endl;
	std::cout << "{" << std::endl;
	std::cout << "\tspeed: " << mReferenceManager->getBaseSpeed() << std::endl;
	std::cout << "\tbase_style: " << mReferenceManager->getBaseStyle().transpose() << std::endl;
	if(constraintSync.size() != 0) {
		std::cout << "\tconstraints_sync: " << std::endl;
		std::cout << "\t{" << std::endl;
		for(int i = 0; i < constraintSync.size(); i++) {
			std::cout << "\t\t" << constraintSync[i].transpose() << std::endl;
		}
		std::cout << "\t}" << std::endl;
	}
	if(constraintStyle.size() != 0) {
		std::cout << "\tconstraints_style: " << std::endl;
		std::cout << "\t{" << std::endl;
		for(int i = 0; i < constraintStyle.size(); i++) {
			std::cout << "\t\t" << std::get<0>(constraintStyle[i])<< " " << std::get<1>(constraintStyle[i]) << " " << std::get<2>(constraintStyle[i]) << std::endl;
		}
		std::cout << "\t}" << std::endl;		
	}

	std::cout << "}" << std::endl;

}