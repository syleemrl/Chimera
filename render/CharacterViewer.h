#ifndef __RENDER_C_VIEWER_WIDGET_H__
#define __RENDER_C_VIEWER_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>

#pragma push_macro("slots")
#undef slots
#include "Camera.h"
#include "GLFunctions.h"
#include "DARTInterface.h"
#include "Functions.h"

#pragma pop_macro("slots")

class CharacterViewer : public QOpenGLWidget
{
	Q_OBJECT

signals:
	void bodyClicked(int);
public slots:

public:
	CharacterViewer();
	void setSkeleton(std::string _skel, std::string _tpose);
	void clearSkeleton();
	std::vector<std::string> getBodyNames();
	void setBodyChecked(int _idx, bool _checked);
	void clearAll();
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

	void drawSkeletons();
	int computeClickedBody(int x, int y);

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	
	dart::dynamics::SkeletonPtr mSkel;
	std::string mSkelPath;
	bool mDrawSkeleton;
	bool mSelectionMode;
};
#endif
