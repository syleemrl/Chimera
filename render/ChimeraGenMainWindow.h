#ifndef __RENDER_CHIMERA_GEN_MAIN_WINDOW_H__
#define __RENDER_CHIMERA_GEN_MAIN_WINDOW_H__
#include <string>
#include <vector>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QListWidget>
#include <QComboBox>

#include "ChimeraGenWidget.h"
#include "RenderConfigParser.h"
#include "CharacterViewer.h"

class ChimeraGenMainWindow : public QMainWindow
{
    Q_OBJECT
    
signals:
	
public slots:
	void togglePlay(const bool& _toggled);
	void changeData(QListWidgetItem* _item);
	void changeSkel(const QString& _str);
	void bodyClicked(QListWidgetItem* _item);
	void bodyClicked(int _idx);
	void addBodyBlock();
	void importSkeleton();
	void exportSkeleton();
	void clearAll();
public:
    ChimeraGenMainWindow();
    ChimeraGenMainWindow(std::vector<RenderData> _renderData);

protected:
	QHBoxLayout* mMainLayout;
	ChimeraGenWidget* mMotionWidget;
	QListWidget* mDataList;
	QListWidget* mBodyList;
	std::vector<bool> mBodyChecked;

	QComboBox* mSkelList;
	CharacterViewer* mCharacterViewer;

	std::vector<RenderData> mRenderData;
	int mDataIdx;

	void initLayoutSetting();
};
#endif