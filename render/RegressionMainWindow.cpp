#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include "RegressionMainWindow.h"

RegressionMainWindow::
RegressionMainWindow() :QMainWindow()
{
    setWindowTitle("Regression renderer");
}
RegressionMainWindow::
RegressionMainWindow(std::vector<RenderData> _renderData)
{   
    RegressionMainWindow();
    initLayoutSetting(_renderData);
}
void
RegressionMainWindow::
initLayoutSetting(std::vector<RenderData> _renderData) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1900,950);
    setMinimumSize(1900,950);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new RegressionViewer(_renderData);
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

    std::vector<std::string> filenames = RenderConfigParser::getFilenames(_renderData);

    mDataList = new QListWidget();
    for(int i = 0; i < filenames.size(); i++) {
        mDataList->addItem(QString::fromStdString(filenames[i]));
    }
    connect(mDataList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(onItemDoubleClicked(QListWidgetItem*)));
    
    paramlayout->addWidget(mDataList);

    QHBoxLayout* editlayout = new QHBoxLayout();
    QVBoxLayout* sliderlayout = new QVBoxLayout();

    for(int i = 0; i < 5; i++) {
        QHBoxLayout* sliderElemlayout = new QHBoxLayout();
        QLabel* label = new QLabel(QString::fromStdString("time "+ std::to_string(i) +":"));
            
        sliderElemlayout->addWidget(label);

        QSlider* param = new QSlider(Qt::Horizontal);
        param->setMinimum(0);
        param->setMaximum(100);
        param->setSingleStep(1);
        param->setProperty("i", i);
        connect (param, SIGNAL(valueChanged(int)), mMotionWidget, SLOT(setTiming(const int&)));
  
        sliderElemlayout->addWidget(param);
        sliderlayout->addLayout(sliderElemlayout);
    }
    editlayout->addLayout(sliderlayout);

    button = new QPushButton("run", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(runRegression())); 
    editlayout->addWidget(button);

    paramlayout->addLayout(editlayout);

    mMainLayout->addLayout(paramlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
}   
void 
RegressionMainWindow::
onItemDoubleClicked(QListWidgetItem* _item)
{
    int idx = mDataList->row(_item);
    mMotionWidget->setRenderData(idx);
}
void 
RegressionMainWindow::
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