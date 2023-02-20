#ifndef __RENDER_SINGLE_MOTION_WIDGET_H__
#define __RENDER_SINGLE_MOTION_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>
#include <QProgressBar>
#include <QLabel>
#include <QString>
#include <QCheckBox>
#pragma push_macro("slots")
#undef slots
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "Camera.h"
#include "GLFunctions.h"
#include "RenderConfigParser.h"
#include "DARTInterface.h"
#include "ReferenceManager.h"
#include "Functions.h"
#include "Controller.h"
#include "SimConfigParser.h"

#pragma pop_macro("slots")

namespace p = boost::python;
namespace np = boost::python::numpy;

class SingleMotionWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	SingleMotionWidget();
	SingleMotionWidget(std::vector<RenderData> _renderData);
	void togglePlay();
	void setRenderData(int _idx);
	void setPhaseWidget(std::vector<QProgressBar*> _pw) {mPhaseWidget = _pw;}
public slots:
	void nextFrame();
	void prevFrame();
	void reset();
protected:
 	void setNetworkSetting(std::string _network);
 	void runPPO(bool _throwBall=false);

	void initializeGL() override;	
	void resizeGL(int w,int h) override;
	void paintGL() override;
	void initLights();

	void timerEvent(QTimerEvent* _event);
	void keyPressEvent(QKeyEvent* _event);

	void mousePressEvent(QMouseEvent* _event);
	void mouseMoveEvent(QMouseEvent* _event);
	void mouseReleaseEvent(QMouseEvent* _event);
	void wheelEvent(QWheelEvent* _event);

	void drawGround();
	void drawSkeletons();
	void setFrame(int _n);

	std::vector<QProgressBar*> mPhaseWidget;
	std::vector<RenderData> mRenderData;

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	bool mTrackCamera;

	bool mPlay;
	int mCurFrame;
	int mTotalFrame;
	int nTimingFuncs;

	std::vector<Eigen::VectorXd> mMotionPPO;
	std::vector<Eigen::VectorXd> mMotionRef;

	std::vector<Eigen::VectorXd> mTiming;

	dart::dynamics::SkeletonPtr mSkelPPO;
	dart::dynamics::SkeletonPtr mSkelRef;

	std::pair<bool, std::vector<Eigen::VectorXd>> mStartDistribution;

	bool mDrawRef;

	p::object mPPO;
	SIM::ReferenceManager* mReferenceManager;
	SIM::Controller* mController;
	SIM::SimConfig mSimConfig;
	p::object mRegressionNet;

	bool mRunSimulation;
	bool mRunRegression;
	bool mSimConfigExists;
	
};
#endif
