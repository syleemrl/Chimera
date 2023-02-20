#ifndef __RENDER_CHIMERA_GEN_WIDGET_H__
#define __RENDER_CHIMERA_GEN_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>
#include "Camera.h"
#include "GLFunctions.h"
#include "RenderConfigParser.h"
#include "DARTInterface.h"
#include "ReferenceManager.h"
#include "SkeletonBuilder.h"
#include "Controller.h"
#pragma push_macro("slots")
#undef slots
#include "Functions.h"
#pragma pop_macro("slots")

class ChimeraGenWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	ChimeraGenWidget();
	ChimeraGenWidget(std::vector<RenderData> _renderData);
	void togglePlay();
	void setRenderData(int _idx);
    void setAssembleMode(bool _on);
    void addBodyBlock(int _skelIdx, std::string _path, std::vector<std::string> _idx);
    void clearBodyBlock();
	void setBodySelected(std::pair<int, int> _idx);
	void assembleBodyBlocks();
	void reposBodyBlocks();
	void loadMotionData();
	void setAssembledSkeleton(std::pair<SIM::SkelInfo, std::map<int, Eigen::Vector3d>> _result);
	SIM::SkelInfo getAssembledSkeleton();
	std::map<int, Eigen::Vector3d> getRotationMap() { return mRotationMap; }
	void clearAll();
	void setSkelNames(std::vector<std::string> _skelNames) { mSkelNames = _skelNames; }
	std::vector<std::string> getSkelNames() { return mSkelNames; }

public slots:
	void nextFrame();
	void prevFrame();
	void reset();
	void fixSkeleton();
	void popBodyBlock();

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
	std::pair<int, int> computeClickedBody(int x, int y);

	void drawGround();
	void drawSkeletons();
	void setFrame(int _n);

	void rotateBody(int _idx, bool _increase);
	void translateBody(int _idx, bool _increase);
	void fixRotation();

	void scaleSelectedBody(bool _up);
	void drawBodyBlocks();

	std::vector<RenderData> mRenderData;

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	bool mTrackCamera;
	
	bool mPlay;
	int mCurFrame;
	int mTotalFrame;

	std::vector<Eigen::VectorXd> mMotionRef;
	std::vector<double> mTiming;

	dart::dynamics::SkeletonPtr mSkelRef;

	std::string mSkelPath;
	std::string mMotionPath;

	SIM::ReferenceManager* mReferenceManager;

	bool mAssembleMode;
	bool mRotationMode;
	bool mPlayMode;

	bool mSelectionMode;
	
	std::vector<std::pair<dart::dynamics::SkeletonPtr, int>> mBodyBlocks;
	std::vector<std::map<int, SIM::BodyInfo>> mBodyBlockMaps;
	std::vector<std::map<int, int>> mIdMaps;

	dart::dynamics::SkeletonPtr mAssembledSkel;
	std::map<int, SIM::BodyInfo> mAssembledMap;
	std::map<int, int> mAssembledIdMap;

	std::vector<std::pair<int, int>> mBodySelected;

	std::vector<dart::dynamics::SkeletonPtr> mBaseSkels;
	std::map<int, Eigen::Vector3d> mRotationMap;
	Eigen::VectorXd mInitialRotation;

	std::vector<std::string> mSkelNames;
	int mIdCount;
	double mUnit;

	bool mSkelLoaded;
};
#endif
