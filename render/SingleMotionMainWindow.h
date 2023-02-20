#ifndef __RENDER_SINGLE_MOTION_MAIN_WINDOW_H__
#define __RENDER_SINGLE_MOTION_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <QTextEdit>
#include <QProgressBar>

#include "SingleMotionWidget.h"
#include "RenderConfigParser.h"

class SingleMotionMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	void onItemDoubleClicked(QListWidgetItem* _item);
public:
    SingleMotionMainWindow();
    SingleMotionMainWindow(std::vector<RenderData> _renderData);

protected:
	QHBoxLayout* mMainLayout;
	SingleMotionWidget* mMotionWidget;
	QListWidget* mDataList;

	void initLayoutSetting(std::vector<RenderData> _renderData);
};
#endif