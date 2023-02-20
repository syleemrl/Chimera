#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "CharacterViewer.h"
#include "SkeletonBuilder.h"
#include "ReferenceManager.h"

CharacterViewer::
CharacterViewer()
:mCamera(new Camera(275, 400)), mDrawSkeleton(false), mSelectionMode(false)
{
	startTimer(30);
	setFocusPolicy( Qt::StrongFocus );
}
void
CharacterViewer::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
CharacterViewer::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
CharacterViewer::
drawSkeletons()
{
	GUI::drawSkeleton(mSkel, 0);
}	
void
CharacterViewer::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	mCamera->apply();

	if(mDrawSkeleton) {
		drawSkeletons();	
	}
}
void
CharacterViewer::
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
CharacterViewer::
timerEvent(QTimerEvent* _event)
{
	update();
}
void
CharacterViewer::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
CharacterViewer::
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
CharacterViewer::
mouseReleaseEvent(QMouseEvent* _event)
{
	if(mSelectionMode) {
		int idx = computeClickedBody(_event->x(), _event->y());

		if(idx != -1) 
			bodyClicked(idx);
	}

	mIsDrag = false;
	mButton = Qt::NoButton;
	update();

}
int
CharacterViewer::
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
	int idx = -1;
	for(int i = 0; i < mSkel->getNumBodyNodes(); i++){

		auto* bn = mSkel->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();

		Eigen::Isometry3d transform = shapeNodes[0]->getTransform();
		Eigen::Vector3d size = dynamic_cast<const dart::dynamics::BoxShape*>(shapeNodes[0]->getShape().get())->getSize() / 2;

		Eigen::Isometry3d T_inv = transform.inverse();

		double t = SIM::boxVecIntersection(size, -size, T_inv*point, T_inv.linear()*dir);
		if( t < 0 ) continue;
		if( t < min ){
			min = t;
			idx = i;
		}
	}

	return idx;
}
void
CharacterViewer::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();

}
void
CharacterViewer::
keyPressEvent(QKeyEvent* _event)
{
	if(_event->key() == Qt::Key_Escape){
		exit(0);
	}
	if(_event->key() == Qt::Key_S){
		mSelectionMode = !mSelectionMode;
		std::cout << "Selection mode : " << mSelectionMode << std::endl;
	}
}
void 
CharacterViewer::
setSkeleton(std::string _skel, std::string _tpose) {
	mSkelPath = std::string(SSC_DIR) + std::string("/data/character/") + _skel;
	mSkel = SIM::SkeletonBuilder::buildFromFile(mSkelPath);

	std::string motionPath = std::string(SSC_DIR) + std::string("/data/motion/") + _tpose;

	SIM::ReferenceManager* rm = new SIM::ReferenceManager(SIM::SkeletonBuilder::buildFromFile(mSkelPath));
	rm->loadMotionFromBVH(motionPath);

	Eigen::VectorXd p = rm->getBaseFrame(0);
	mSkel->setPositions(p);

	delete rm;


	GUI::setSkeletonColor(mSkel, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));

	mDrawSkeleton = true;

	mCamera->reset();
	Eigen::Vector3d com = mSkel->getRootBodyNode()->getCOM();
	mCamera->setCenter(com);
}
void 
CharacterViewer::
clearSkeleton() {
	mDrawSkeleton = false;
}
void
CharacterViewer::
clearAll() {
	clearSkeleton();
}
std::vector<std::string> 
CharacterViewer::
getBodyNames() {
	std::vector<std::string> result;
	for(int i = 0; i < mSkel->getNumBodyNodes(); i++) {
		result.push_back(mSkel->getBodyNode(i)->getName());
	}
	return result;
}
void 
CharacterViewer::
setBodyChecked(int _idx, bool _checked) {
	dart::dynamics::BodyNode* bn = mSkel->getBodyNode(_idx);
	if(_checked) {
		GUI::setBodyColor(bn, Eigen::Vector4d(73./255., 73./255., 235./255., 1.0));	
	} else {
		GUI::setBodyColor(bn, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));	
	}
}
