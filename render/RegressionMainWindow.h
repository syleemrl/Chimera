#ifndef __RENDER_REGRESSION_MAIN_WINDOW_H__
#define __RENDER_REGRESSION_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <QTextEdit>
#include <QProgressBar>

#include "RegressionViewer.h"
#include "RenderConfigParser.h"

class RegressionMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void onItemDoubleClicked(QListWidgetItem* _item);
	void togglePlay(const bool& _toggled);

public:
    RegressionMainWindow();
    RegressionMainWindow(std::vector<RenderData> _renderData);

protected:
	QHBoxLayout* mMainLayout;
	RegressionViewer* mMotionWidget;
	QListWidget* mDataList;

	void initLayoutSetting(std::vector<RenderData> _renderData);
};
#endif