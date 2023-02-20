#ifndef __RENDER_REGRESSION_VIEWER_H__
#define __RENDER_REGRESSION_VIEWER_H__
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
#include "SimConfigParser.h"

#pragma pop_macro("slots")

namespace p = boost::python;
namespace np = boost::python::numpy;

class RegressionViewer : public QOpenGLWidget
{
	Q_OBJECT

public:
	RegressionViewer();
	RegressionViewer(std::vector<RenderData> _renderData);
	void togglePlay();
	void setRenderData(int _idx);
	int getNumTimingFunc() { return nTimingFuncs; }
public slots:
	void nextFrame();
	void prevFrame();
	void reset();
	void runRegression();
	void setTiming(const int& _value);
protected:
 	void setNetworkSetting(std::string _network);

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

	std::vector<RenderData> mRenderData;

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	int nTimingFuncs;

	bool mPlay;
	int mCurFrame;
	int mTotalFrame;
	Eigen::VectorXd mInputPhase;

	std::vector<Eigen::VectorXd> mMotionIK;
	std::vector<Eigen::VectorXd> mMotionNet;
	std::vector<Eigen::VectorXd> mTiming;

	dart::dynamics::SkeletonPtr mSkelIK;
	dart::dynamics::SkeletonPtr mSkelNet;

	std::string mSkelPath;
	std::string mMotionPath;

	SIM::ReferenceManager* mReferenceManager;
	SIM::SimConfig mSimConfig;
	p::object mRegressionNet;

	bool mDrawNet;
	bool mDrawIK;

	int mOverhead=60;
};
#endif
