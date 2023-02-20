#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "ChimeraGenWidget.h"
#include "SkeletonBuilder.h"

ChimeraGenWidget::
ChimeraGenWidget()
:mCamera(new Camera(1000, 700)), mCurFrame(0), mTotalFrame(0), mPlay(false), mTrackCamera(false)
, mSkelLoaded(false), mAssembleMode(true), mSelectionMode(false), mRotationMode(false), mPlayMode(false), mIdCount(0), mUnit(0.1)
{
	startTimer(30);
}
ChimeraGenWidget::
ChimeraGenWidget(std::vector<RenderData> _renderData)
:ChimeraGenWidget()
{
	mRenderData = _renderData;

	setFocusPolicy( Qt::StrongFocus );
}
void
ChimeraGenWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
ChimeraGenWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
ChimeraGenWidget::
setFrame(int n)
{
	mSkelRef->setPositions(mMotionRef[n]);
}
void
ChimeraGenWidget::
drawSkeletons()
{
	GUI::drawSkeleton(mSkelRef, 0);
}	
void
ChimeraGenWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelRef->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void 
ChimeraGenWidget::
drawBodyBlocks()
{
	if(!mAssembleMode) {
		GUI::drawSkeleton(mAssembledSkel, 0);
		for(int i = 0; i < mAssembledSkel->getNumBodyNodes(); i++) {
			Eigen::Vector3d pos = mAssembledSkel->getBodyNode(i)->getTransform().translation();
			Eigen::Matrix3d rot = mAssembledSkel->getBodyNode(i)->getTransform().linear();

			GUI::drawLine(pos, pos+0.2*rot.col(0), Eigen::Vector3d(1, 0, 0));
			GUI::drawLine(pos, pos+0.2*rot.col(1), Eigen::Vector3d(0, 1, 0));
			GUI::drawLine(pos, pos+0.2*rot.col(2), Eigen::Vector3d(0, 0, 1));	
		}
	} else {
		for(int i = 0; i < mBodyBlocks.size(); i++) {
			GUI::drawSkeleton(mBodyBlocks[i].first, 0);
		}
	}
}
void
ChimeraGenWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);
	
	if(mAssembleMode || mRotationMode) {
		mCamera->apply();

		drawBodyBlocks();

		if(mAssembleMode) {
			GUI::drawStringOnScreen(0.02, 0.95, "Assemble mode", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.92, "Select enable: S", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.89, "Assemble: A", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.86, "Scale Up: 0", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.83, "Scale Down: P", true, Eigen::Vector3d::Zero());
		} else if(mRotationMode) {
			GUI::drawStringOnScreen(0.02, 0.95, "Rotation mode", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.92, "Select enable: S", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.89, "Rotate: 123QWE", true, Eigen::Vector3d::Zero());
			GUI::drawStringOnScreen(0.02, 0.86, "Translate: 456RTY", true, Eigen::Vector3d::Zero());
		}

	} else {
		if(mTrackCamera){
			Eigen::Vector3d com = mSkelRef->getRootBodyNode()->getCOM();
			Eigen::Isometry3d transform = mSkelRef->getRootBodyNode()->getTransform();
			com[1] = 0.8;
			mCamera->setCenter(com);
		}
		mCamera->apply();

		drawGround();
		drawSkeletons();

		GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
	}
}
void
ChimeraGenWidget::
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
ChimeraGenWidget::
timerEvent(QTimerEvent* _event)
{
	if(!mAssembleMode && !mRotationMode) {
		if(mPlay && mCurFrame < mTotalFrame) {
			mCurFrame += 1;
		} 
		setFrame(mCurFrame);
	}
	update();

}
void
ChimeraGenWidget::
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
	if(_event->key() == Qt::Key_S){
		mSelectionMode = !mSelectionMode;
		std::cout << "Selection mode : " << mSelectionMode << std::endl;
	}
	if(_event->key() == Qt::Key_A && mSelectionMode && mAssembleMode){
		std::cout << "Assemble two body" << std::endl;
		assembleBodyBlocks();
	}
	if(_event->key() == Qt::Key_1 && mRotationMode && mBodySelected.size() == 1) {
		rotateBody(1, true);
	}
	if(_event->key() == Qt::Key_2 && mRotationMode && mBodySelected.size() == 1) {
		rotateBody(2, true);
	}
	if(_event->key() == Qt::Key_3 && mRotationMode && mBodySelected.size() == 1) {
		rotateBody(3, true);
	}
	if(_event->key() == Qt::Key_Q && mRotationMode && mBodySelected.size() == 1) {
		rotateBody(1, false);
	}
	if(_event->key() == Qt::Key_W && mRotationMode && mBodySelected.size() == 1) {
		rotateBody(2, false);
	}
	if(_event->key() == Qt::Key_E && mRotationMode && mBodySelected.size() == 1) {
		rotateBody(3, false);
	}
	if(_event->key() == Qt::Key_4 && mRotationMode && mBodySelected.size() == 1) {
		translateBody(1, true);
	}
	if(_event->key() == Qt::Key_5 && mRotationMode && mBodySelected.size() == 1) {
		translateBody(2, true);
	}
	if(_event->key() == Qt::Key_6 && mRotationMode && mBodySelected.size() == 1) {
		translateBody(3, true);
	}
	if(_event->key() == Qt::Key_R && mRotationMode && mBodySelected.size() == 1) {
		translateBody(1, false);
	}
	if(_event->key() == Qt::Key_T && mRotationMode && mBodySelected.size() == 1) {
		translateBody(2, false);
	}
	if(_event->key() == Qt::Key_Y && mRotationMode && mBodySelected.size() == 1) {
		translateBody(3, false);
	}	
	if(_event->key() == Qt::Key_0 && mAssembleMode && mBodySelected.size() == 1) {
		scaleSelectedBody(true);
	}
	if(_event->key() == Qt::Key_P && mAssembleMode && mBodySelected.size() == 1) {
		scaleSelectedBody(false);
	}	
	if(_event->key() == Qt::Key_N && mRotationMode) {
		if(mUnit > 0.01)
			mUnit -= 0.01;
		std::cout << "rotation unit: " << mUnit << std::endl;
	}	
	if(_event->key() == Qt::Key_M && mRotationMode) {
		mUnit += 0.01;
		std::cout << "rotation unit: " << mUnit << std::endl;
	}	
}
void
ChimeraGenWidget::
assembleBodyBlocks()
{
	if(mBodySelected.size() != 2) {
		std::cout << "2 blocks should be selected to assemble." << std::endl;
		return;
	}
	std::pair<int, int> body0 = mBodySelected[0];
	std::pair<int, int> body1 = mBodySelected[1];

	int idx0 = body0.first;
	int idx1 = body1.first;

	setBodySelected(body0);
	setBodySelected(body1);

	std::string skel1Path = std::string(SSC_DIR) + std::string("/data/character/") + mSkelNames[mBodyBlockMaps[idx1][0].skelIdx];
	int numBodiesBefore = mBodyBlocks[idx0].first->getNumBodyNodes();
	dart::dynamics::SkeletonPtr skel = SIM::SkeletonBuilder::assembleBodyBlocks(mBodyBlocks[idx0].first, body0.second,
																	mBodyBlocks[idx1].first, mBodyBlockMaps[idx1], mBodyBlockMaps[idx1][0].skelBodyIdx,
																	skel1Path);
	
	std::map<int, int> idMap = mIdMaps[idx0];
	std::map<int, SIM::BodyInfo> bodyMap = mBodyBlockMaps[idx0];
	for(int i = numBodiesBefore; i < skel->getNumBodyNodes(); i++) {
		std::string name = skel->getBodyNode(i)->getName();
		for(int j = 0; j < mBodyBlocks[idx1].first->getNumBodyNodes(); j++) {
			std::string nameOriginal = mBodyBlocks[idx1].first->getBodyNode(j)->getName();
			if(name == nameOriginal) {
				idMap.insert(std::pair<int, int>(i, mIdMaps[idx1][j]));
				bodyMap.insert(std::pair<int, SIM::BodyInfo>(i, mBodyBlockMaps[idx1][j]));
				break;
			}
		}
	}


	if(idx0 > idx1) {
		mBodyBlocks.erase(mBodyBlocks.begin() + idx0);
		mBodyBlocks.erase(mBodyBlocks.begin() + idx1);

		mBodyBlockMaps.erase(mBodyBlockMaps.begin() + idx0);
		mBodyBlockMaps.erase(mBodyBlockMaps.begin() + idx1);

		mIdMaps.erase(mIdMaps.begin() + idx0);
		mIdMaps.erase(mIdMaps.begin() + idx1);

	} else {
		mBodyBlocks.erase(mBodyBlocks.begin() + idx1);
		mBodyBlocks.erase(mBodyBlocks.begin() + idx0);
		
		mBodyBlockMaps.erase(mBodyBlockMaps.begin() + idx1);
		mBodyBlockMaps.erase(mBodyBlockMaps.begin() + idx0);

		mIdMaps.erase(mIdMaps.begin() + idx1);
		mIdMaps.erase(mIdMaps.begin() + idx0);

	}

	Eigen::VectorXd pos = skel->getPositions();

	for(int i = 0; i < skel->getNumBodyNodes(); i++) {
		SIM::BodyInfo bi = bodyMap[i];

		int skelIdx = bi.skelIdx;
		int bodyIdx = bi.skelBodyIdx;

		Eigen::VectorXd p = mBaseSkels[skelIdx]->getPositions();

		if(i == 0) {
			if(bodyIdx == 0)
				pos.segment<3>(0) = p.segment<3>(0);
			else {
				Eigen::Isometry3d transform = mBaseSkels[skelIdx]->getBodyNode(bodyIdx)->getWorldTransform();
				pos.segment<3>(0) = SIM::logMap(transform.linear());
			}

		} else {
			if(bodyIdx == 0)
				pos.segment<3>(3 + 3 * i) = p.segment<3>(0);
			else
				pos.segment<3>(3 + 3 * i) = p.segment<3>(3 + 3 * bodyIdx);
		}
	}
	SIM::SkeletonBuilder::scaleBodyNode(mBaseSkels, skel, bodyMap);

	skel->setPositions(pos);

	mBodyBlocks.push_back(std::pair<dart::dynamics::SkeletonPtr, int>(skel, 0));	
	mBodyBlockMaps.push_back(bodyMap);	
	mIdMaps.push_back(idMap);

	reposBodyBlocks();	
}
void
ChimeraGenWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
ChimeraGenWidget::
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
ChimeraGenWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	if(mSelectionMode && (mAssembleMode || mRotationMode)) {
		std::pair<int, int> idx = computeClickedBody(_event->x(), _event->y());
		if(idx.first != -1)
			setBodySelected(idx);
	} 
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
std::pair<int, int> 
ChimeraGenWidget::
computeClickedBody(int x, int y)
{
	makeCurrent();
	
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLdouble winX, winY, winZ;
	GLdouble posX, posY, posZ;

	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );

	winX = (float)x;
	winY = (float)viewport[3] - (float)y;

	gluUnProject( winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	Eigen::Vector3d point = Eigen::Vector3d(posX, posY, posZ);

	gluUnProject( winX, winY, 0.99, modelview, projection, viewport, &posX, &posY, &posZ);

	Eigen::Vector3d dir = Eigen::Vector3d(posX, posY, posZ) - point;
	dir.normalize();

	double min = 9999;
	int idxSkel = -1;
	int idxBody = -1;

	for(int i = 0; i < mBodyBlocks.size(); i++) {
		for(int j = 0; j < mBodyBlocks[i].first->getNumBodyNodes(); j++){

			auto* bn = mBodyBlocks[i].first->getBodyNode(j);
			auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();

			Eigen::Isometry3d transform = shapeNodes[0]->getTransform();
			Eigen::Vector3d size = dynamic_cast<const dart::dynamics::BoxShape*>(shapeNodes[0]->getShape().get())->getSize() / 2;

			Eigen::Isometry3d T_inv = transform.inverse();

			double t = SIM::boxVecIntersection(size, -size, T_inv*point, T_inv.linear()*dir);
			if( t < 0 ) continue;
			if( t < min ){
				min = t;
				idxSkel = i;
				idxBody = j;
			}
		}
	}

	if(!mSkelLoaded && mRotationMode && idxSkel != -1 && idxBody != -1) {
		while(1) {
			if(idxBody == 0) 
				break;

			auto bnParent = mAssembledSkel->getBodyNode(idxBody)->getParentBodyNode();
			int idxParent = bnParent->getIndexInSkeleton();

			if(mAssembledIdMap[idxBody] != mAssembledIdMap[idxParent])
			   break;
			idxBody = idxParent;
		}
	}

	return std::pair<int, int> (idxSkel, idxBody);
}
void
ChimeraGenWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
ChimeraGenWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
ChimeraGenWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
ChimeraGenWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
ChimeraGenWidget::
togglePlay() {
	mPlay = !mPlay;
}
void 
ChimeraGenWidget::
setRenderData(int _idx) {

	RenderData data = mRenderData[_idx];

	mBaseSkels.clear();
	for(int i = 0 ; i < data.CMpair.size(); i++) {
		std::string skelPath = std::string(SSC_DIR) + std::string("/data/character/") + data.CMpair[i].first;
		dart::dynamics::SkeletonPtr skel = SIM::SkeletonBuilder::buildFromFile(skelPath);
		mBaseSkels.push_back(skel);
	}

	std::vector<dart::dynamics::SkeletonPtr> skels;
	for(int i = 0 ; i < data.CMpair.size(); i++) {
		std::string skelPath = std::string(SSC_DIR) + std::string("/data/character/") + data.CMpair[i].first;
		skels.push_back(SIM::SkeletonBuilder::buildFromFile(skelPath));
	}
	mReferenceManager = new SIM::ReferenceManager(skels);

	for(int i = 0 ; i < data.CMpair.size(); i++) {
		std::string motionPath = std::string(SSC_DIR) + std::string("/data/motion/") + data.CMpair[i].second;
		mReferenceManager->loadMotionFromBVH(motionPath, i);
		mBaseSkels[i]->setPositions(mReferenceManager->getBaseFrame(0, i));
	}

}
void
ChimeraGenWidget::
setAssembleMode(bool _on) 
{
	mAssembleMode = _on;
}
void
ChimeraGenWidget::
clearBodyBlock()
{
	mSelectionMode = false;

	std::vector<std::pair<int, int>> bodySelectedTemp;
	for(int i = 0; i < mBodySelected.size(); i++)
		bodySelectedTemp.push_back(mBodySelected[i]);

	for(int i = 0; i < bodySelectedTemp.size(); i++)
		setBodySelected(bodySelectedTemp[i]);

	mBodyBlocks.clear();
	mBodyBlockMaps.clear();
	mIdMaps.clear();
}
void 
ChimeraGenWidget::
clearAll()
{
	clearBodyBlock();

	mAssembleMode = true;
	mSelectionMode = false;
	mRotationMode = false;
	mPlayMode = false;

	delete mReferenceManager;

}
void
ChimeraGenWidget::
addBodyBlock(int _skelIdx, std::string _path, std::vector<std::string> _idx)
{
	std::string path = std::string(SSC_DIR) + std::string("/data/character/") + _path;
	SIM::SkelInfo result = SIM::SkeletonBuilder::buildBodyBlock(_skelIdx, path, _idx);
	GUI::setSkeletonColor(result.skel, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));

	Eigen::VectorXd pos = result.skel->getPositions();

	for(int i = 0; i < result.skel->getNumBodyNodes(); i++) {
		int skelIdx = result.bodyMap[i].skelIdx;
		int bodyIdx = result.bodyMap[i].skelBodyIdx;
		result.bodyMap[i].scale = 1.0;

		Eigen::VectorXd p = mReferenceManager->getBaseFrame(0, skelIdx);
		if(i == 0) {
			if(bodyIdx == 0) {
					pos.segment<3>(0) = p.segment<3>(0);
			} else {
					Eigen::Isometry3d transform = mBaseSkels[skelIdx]->getBodyNode(bodyIdx)->getWorldTransform();
					pos.segment<3>(0) = SIM::logMap(transform.linear());
			}

		} else {
			if(bodyIdx == 0)
				pos.segment<3>(3 + 3 * i) = p.segment<3>(0);
			else
				pos.segment<3>(3 + 3 * i) = p.segment<3>(3 + 3 * bodyIdx);
		}
	}


	double dx = mBodyBlocks.size() % 2 == 0? -1: 1;
	double dy = 1.5 - mBodyBlocks.size() / 2;

	pos.segment<3>(3) = dx * Eigen::Vector3d(1, 0, 0) + dy * Eigen::Vector3d(0, 1, 0);
	result.skel->setPositions(pos);

	mBodyBlocks.push_back(std::pair<dart::dynamics::SkeletonPtr, int>(result.skel, mIdCount));
	mBodyBlockMaps.push_back(result.bodyMap);

	std::map<int, int> idMap;
	for(int i = 0; i < result.skel->getNumBodyNodes(); i++) {
		idMap.insert(std::pair<int, int>(i, mIdCount));
	}
	mIdMaps.push_back(idMap);
	mIdCount += 1;
}
void
ChimeraGenWidget::
popBodyBlock()
{
	if(mBodyBlocks.size() != 0)  {
		mBodyBlocks.pop_back();
		mBodyBlockMaps.pop_back();
		mIdMaps.pop_back();
	}

}
void
ChimeraGenWidget::
setBodySelected(std::pair<int, int> _idx)
{
	dart::dynamics::BodyNode* bn = mBodyBlocks[_idx.first].first->getBodyNode(_idx.second);
	auto it = std::find(mBodySelected.begin(), mBodySelected.end(), _idx);

	if(it != mBodySelected.end()) {
		mBodySelected.erase(it);
		GUI::setBodyColor(bn, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));	
	} else {
		if(mRotationMode && mBodySelected.size() != 0)
			return;

		mBodySelected.push_back(_idx);
		GUI::setBodyColor(bn, Eigen::Vector4d(73./255., 73./255., 235./255., 1.0));	

	}
}
void
ChimeraGenWidget::
reposBodyBlocks()
{
	for(int i = 0; i < mBodyBlocks.size(); i++) {
		Eigen::VectorXd pos = mBodyBlocks[i].first->getPositions();
		double dx = i % 2 == 0? -1: 1;
		double dy = 1.5 - i / 2;

		pos.segment<3>(3) = dx * Eigen::Vector3d(1, 0, 0) + dy * Eigen::Vector3d(0, 1, 0);
		mBodyBlocks[i].first->setPositions(pos);
	}
}
void
ChimeraGenWidget::
fixSkeleton()
{
	if(mPlayMode) {
		mPlayMode = false;
		mRotationMode = true;

		return;
	}
	if(mRotationMode) {
		fixRotation();
		return;
	}

	mAssembleMode = false;
	mRotationMode = true;

	if(!mAssembledSkel) {
		mAssembledSkel = mBodyBlocks[0].first;
		mAssembledMap = mBodyBlockMaps[0];
		mAssembledIdMap = mIdMaps[0];
	}
	
	mInitialRotation = mAssembledSkel->getPositions();
	mReferenceManager->setCurrentCharacter(mAssembledSkel->cloneSkeleton(), mAssembledMap);
	mReferenceManager->setTimeStep();

	mSkelRef = mAssembledSkel->cloneSkeleton();

	GUI::setSkeletonColor(mSkelRef, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));
}
void
ChimeraGenWidget::
fixRotation()
{
	mRotationMode = false;
	mRotationMap.clear();

	for(int i = 0; i < mAssembledSkel->getNumBodyNodes(); i++) {
		int idx = mAssembledSkel->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
		Eigen::VectorXd curPosition = mAssembledSkel->getPositions();
		if((mInitialRotation.segment<3>(idx) - curPosition.segment<3>(idx)).norm() > 1e-8) {
			Eigen::Vector3d diff = SIM::jointPositionDifferences(curPosition.segment<3>(idx), mInitialRotation.segment<3>(idx));

			if(mRotationMap.find(i) != mRotationMap.end())
				mRotationMap[i] = diff;
			else
				mRotationMap.insert(std::pair<int, Eigen::Vector3d>(i, diff));
		}
	}
	mPlayMode = true;

	loadMotionData();
}
void
ChimeraGenWidget::
rotateBody(int _idx, bool _increase)
{


	int idx = mAssembledSkel->getBodyNode(mBodySelected[0].second)->getParentJoint()->getIndexInSkeleton(0);
	Eigen::VectorXd pos =  mAssembledSkel->getPositions();
	Eigen::Vector3d n = pos.segment<3>(idx);

	Eigen::Vector3d rotation;
	rotation.setZero();

	if(_increase)
		rotation(_idx-1) += mUnit;
	else
		rotation(_idx-1) -= mUnit;	

	n = SIM::rotate(rotation, n);
	pos.segment<3>(idx) = n;

	mAssembledSkel->setPositions(pos);
}
void 
ChimeraGenWidget::
translateBody(int _idx, bool _increase)
{
	auto bn = mAssembledSkel->getBodyNode(mBodySelected[0].second);
	auto props = bn->getParentJoint()->getJointProperties();
	Eigen::Vector3d n = props.mT_ParentBodyToJoint.translation();
	if(_increase)
		n(_idx-1) += mUnit * 0.2;
	else
		n(_idx-1) -= mUnit * 0.2;	

	props.mT_ParentBodyToJoint.translation() = n;
	bn->getParentJoint()->setProperties(props);

	bn = mSkelRef->getBodyNode(mBodySelected[0].second);
	props = bn->getParentJoint()->getJointProperties();
	props.mT_ParentBodyToJoint.translation() = n;
	bn->getParentJoint()->setProperties(props);
}

void
ChimeraGenWidget::
loadMotionData()
{
	mCurFrame = 0;
	mTotalFrame = 0;

	Eigen::VectorXd phase(mReferenceManager->getNumTimingFunc());
	phase.setZero();

	bool endFlag = false;
	mMotionRef.clear();
	while(1) {
		Eigen::VectorXd p = mReferenceManager->getFrame(phase);

		for(int i = 0; i < mAssembledSkel->getNumBodyNodes(); i++) {
			if(mRotationMap.find(i) != mRotationMap.end()) {
				Eigen::Vector3d rot = mRotationMap[i];
				int idx = mAssembledSkel->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
				p.segment<3>(idx) = SIM::rotate(p.segment<3>(idx), rot);
			}
		}
		mMotionRef.push_back(p);

		phase += mReferenceManager->getTimeStep(phase);

		for(int i = 0; i < mReferenceManager->getNumTimingFunc(); i++) {
			if(phase[i] >= mReferenceManager->getMotionLength(i)) {
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
ChimeraGenWidget::
setAssembledSkeleton(std::pair<SIM::SkelInfo, std::map<int, Eigen::Vector3d>> _result)
{
	mSkelLoaded = true;
	mPlayMode = true;
	mSelectionMode = false;
	mAssembleMode = false;

	mAssembledSkel = _result.first.skel;
	mAssembledMap = _result.first.bodyMap;
	mRotationMap = _result.second;

	mBodyBlocks.push_back(std::pair<dart::dynamics::SkeletonPtr, int>(mAssembledSkel, 0));
	mBodyBlockMaps.push_back(mAssembledMap);

	SIM::SkeletonBuilder::scaleBodyNode(mBaseSkels, mAssembledSkel, mAssembledMap);

	mReferenceManager->setCurrentCharacter(mAssembledSkel->cloneSkeleton(), mAssembledMap);
	mReferenceManager->setTimeStep();
	mSkelRef = mAssembledSkel->cloneSkeleton();

	GUI::setSkeletonColor(mSkelRef, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));

	loadMotionData();
	

	Eigen::VectorXd phase(mReferenceManager->getNumTimingFunc());
	phase.setZero();

	Eigen::VectorXd p = mReferenceManager->getFrame(phase);
	mInitialRotation = p;
	for(int i = 0; i < mAssembledSkel->getNumBodyNodes(); i++) {
		if(mRotationMap.find(i) != mRotationMap.end()) {
			Eigen::Vector3d rot = mRotationMap[i];
			int idx = mAssembledSkel->getBodyNode(i)->getParentJoint()->getIndexInSkeleton(0);
			p.segment<3>(idx) = SIM::rotate(p.segment<3>(idx), rot);
		}
	}
	mAssembledSkel->setPositions(p);

}
SIM::SkelInfo
ChimeraGenWidget::
getAssembledSkeleton()
{
	SIM::SkelInfo info;
	info.skel = mBodyBlocks[0].first;
	info.bodyMap = mBodyBlockMaps[0];

	return info;
}
void 
ChimeraGenWidget::
scaleSelectedBody(bool _up)
{
	int idx = mBodySelected[0].first;
	for(auto it = mBodyBlockMaps[idx].begin(); it != mBodyBlockMaps[idx].end(); it++) {
		if(_up)
			it->second.scale += 0.05;
		else
			it->second.scale -= 0.05;
	}
	SIM::SkeletonBuilder::scaleBodyNode(mBaseSkels, mBodyBlocks[idx].first, mBodyBlockMaps[idx]);
}