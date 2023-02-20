#ifndef __RENDER_ASSEMBLER_WIDGET_H__
#define __RENDER_ASSEMBLER_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>
#include <QProgressBar>
#include <QLabel>
#include <QString>
#include <QSlider>
#include <mutex>
#pragma push_macro("slots")
#undef slots
#include "Camera.h"
#include "GLFunctions.h"
#include "RenderConfigParser.h"
#include "DARTInterface.h"
#include "ReferenceManager.h"
#include "Functions.h"
#include "Controller.h"
#include "SimConfigParser.h"

#pragma pop_macro("slots")

class AssemblerWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	AssemblerWidget();
	AssemblerWidget(std::vector<RenderData> _renderData);
	void togglePlay();
	void setRenderData(int _idx);
	void saveMotion(std::string _path, int _start, int _end);
	void setPhaseWidgets(std::vector<QProgressBar*> _pw) {mPhaseWidgets = _pw;}
    void setTimeline(std::vector<QSlider*> _tl) {mTimelines = _tl;}    
    Eigen::VectorXd getInputPhase() { return mInputPhase; }

public slots:
	void nextFrame();
	void prevFrame();
	void reset();
	void editMotion();
	void setPreview();
	void setPhase(const int& _value);
	void setStyle(const int& _value);
	void setSpeed(const int& _value);
    std::string addConstraint();
	void styleActivated();
	void speedActivated();
	void phaseActivated();
    void setSliderCurrent();  
    void printCurrentSettings();
protected:
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

	std::vector<QProgressBar*> mPhaseWidgets;
	std::vector<QSlider*> mTimelines;
	std::vector<RenderData> mRenderData;

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	bool mTrackCamera;

	bool mPlayMode;
	bool mEditMode;

	int mCurFrame;
	int mTotalFrame;
	int nTimingFuncs;

	std::vector<Eigen::VectorXd> mMotionRef;
	std::vector<Eigen::VectorXd> mTiming;
	dart::dynamics::SkeletonPtr mSkelRef;

	std::vector<std::vector<Eigen::VectorXd>> mMotionRefBase;
	std::vector<dart::dynamics::SkeletonPtr> mSkelRefBase;

	SIM::ReferenceManager* mReferenceManager;
	SIM::SimConfig mSimConfig;

	Eigen::VectorXd mMotionRefPreview;
	std::vector<Eigen::VectorXd> mMotionRefBasePreview;

	bool mSpeedActivated;
	std::vector<bool> mStyleActivated;
	std::vector<bool> mPhaseActivated;

	double mInputSpeed;
	
	Eigen::VectorXd mInputStyle;
	Eigen::VectorXd mInputPhase;

};
#endif
