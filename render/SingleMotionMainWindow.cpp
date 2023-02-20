#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include "SingleMotionMainWindow.h"

SingleMotionMainWindow::
SingleMotionMainWindow() :QMainWindow()
{
    setWindowTitle("Single Character Single Motion renderer");
}
SingleMotionMainWindow::
SingleMotionMainWindow(std::vector<RenderData> _renderData)
{   
    SingleMotionMainWindow();
    initLayoutSetting(_renderData);
}
void
SingleMotionMainWindow::
initLayoutSetting(std::vector<RenderData> _renderData) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1900,950);
    setMinimumSize(1900,950);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new SingleMotionWidget(_renderData);
    mMotionWidget->setMinimumSize(1600,900);
    mMotionWidget->setMaximumSize(1600,900);

    motionlayout->addWidget(mMotionWidget);

    QHBoxLayout *buttonlayout = new QHBoxLayout();
    buttonlayout->addStretch(1);

    QPushButton* button = new QPushButton("reset", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(reset())); 
    buttonlayout->addWidget(button);
    
    button = new QPushButton("prev", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(prevFrame())); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("play", this);
    button->setCheckable(true);
    connect(button, SIGNAL(toggled(bool)), this, SLOT(togglePlay(const bool&))); 
    buttonlayout->addWidget(button); 

    button = new QPushButton("next", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(nextFrame())); 
    buttonlayout->addWidget(button);    
    buttonlayout->addStretch(1);

    motionlayout->addLayout(buttonlayout);
    mMainLayout->addLayout(motionlayout);
 
    QVBoxLayout* paramlayout = new QVBoxLayout();
 
    QVBoxLayout* phaseLayout = new QVBoxLayout();

    std::vector<QProgressBar*> phaseWidget;

    for(int i = 0; i < 5; i++) {
        QHBoxLayout* phaseRowLayout = new QHBoxLayout();

        QProgressBar* phaseBar = new QProgressBar();

        phaseBar->setMaximum(1000);
        phaseBar->setValue(0);
        phaseBar->setTextVisible(false);
        phaseBar->setStyleSheet("background-color:#EFEAE8;");

        QHBoxLayout *layout = new QHBoxLayout(phaseBar);
        QLabel *overlay = new QLabel();
        overlay->setAlignment(Qt::AlignCenter);
        overlay->setText("");
        overlay->setStyleSheet("color:#EFEAE8; background-color: rgba(0,0,0,0%)");

        layout->addWidget(overlay);
        layout->setContentsMargins(0,0,0,0);

        // phaseRowLayout->addWidget(phaseBar);  

        phaseLayout->addWidget(phaseBar);

        phaseWidget.push_back(phaseBar);

    }    
    mMotionWidget->setPhaseWidget(phaseWidget);

    paramlayout->addLayout(phaseLayout);
    paramlayout->addStretch(1);

    std::vector<std::string> filenames = RenderConfigParser::getFilenames(_renderData);

    mDataList = new QListWidget();
    for(int i = 0; i < filenames.size(); i++) {
        mDataList->addItem(QString::fromStdString(filenames[i]));
    }
    connect(mDataList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
    
    paramlayout->addWidget(mDataList);
    paramlayout->addStretch(1);

    mMainLayout->addLayout(paramlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
}
void 
SingleMotionMainWindow::
togglePlay(const bool& _toggled)
{
    auto button = qobject_cast<QPushButton*>(sender());
    if(_toggled) {
        button->setText("pause");
    } else {
        button->setText("play");
    }
    mMotionWidget->togglePlay();
}    
void 
SingleMotionMainWindow::
onItemDoubleClicked(QListWidgetItem* _item)
{
    int idx = mDataList->row(_item);
    mMotionWidget->setRenderData(idx);
}