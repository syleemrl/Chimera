#ifndef __RENDER_ASSEMBLER_MAIN_WINDOW_H__
#define __RENDER_ASSEMBLER_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <QTextEdit>
#include <QProgressBar>

#include "AssemblerWidget.h"
#include "RenderConfigParser.h"

class AssemblerMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	void onItemDoubleClicked(QListWidgetItem* _item);
	void addConstraint();
public:
    AssemblerMainWindow();
    AssemblerMainWindow(std::vector<RenderData> _renderData);

protected:
	QHBoxLayout* mMainLayout;
	AssemblerWidget* mMotionWidget;
	QListWidget* mDataList;
	QListWidget* mConstraintList;

	void initLayoutSetting(std::vector<RenderData> _renderData);
};
#endif