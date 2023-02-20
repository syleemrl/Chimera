#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include <QGroupBox>
#include "AssemblerMainWindow.h"

AssemblerMainWindow::
AssemblerMainWindow() :QMainWindow()
{
    setWindowTitle("Synthesis renderer");
}
AssemblerMainWindow::
AssemblerMainWindow(std::vector<RenderData> _renderData)
{   
    AssemblerMainWindow();
    initLayoutSetting(_renderData);
}
void
AssemblerMainWindow::
initLayoutSetting(std::vector<RenderData> _renderData) {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1900,950);
    setMinimumSize(1900,950);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new AssemblerWidget(_renderData);
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

    QVBoxLayout* phaseLayout = new QVBoxLayout();
    // suppose maximum num phase function = 4
    std::vector<QProgressBar*> phaseWidgets;

    for(int i = 0; i < 5; i++) {
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

        phaseLayout->addWidget(phaseBar);
        phaseWidgets.push_back(phaseBar);

    }    
    mMotionWidget->setPhaseWidgets(phaseWidgets);

    paramlayout->addLayout(phaseLayout);
    paramlayout->addStretch(1);


    /////////// speed
    QGroupBox* speedGroup = new QGroupBox();

    QHBoxLayout* speedElemLayout = new QHBoxLayout();
    // speedElemLayout->setStyleSheet("background-color:#EFEAE8;");

    QCheckBox* speedBox = new QCheckBox();
    speedBox->setCheckable(true);
    connect(speedBox, SIGNAL(clicked()), mMotionWidget, SLOT(speedActivated())); 
    speedElemLayout->addWidget(speedBox);

    QLabel* label = new QLabel(QString::fromStdString("speed"));
    speedElemLayout->addWidget(label);
    
    QSlider* param = new QSlider(Qt::Horizontal);
    param->setMinimum(0);
    param->setMaximum(10);
    param->setSingleStep(1);
        
    connect (param, SIGNAL(valueChanged(int)), mMotionWidget, SLOT(setSpeed(const int&)));
     
    speedElemLayout->addWidget(param);
    speedGroup->setLayout(speedElemLayout);
    paramlayout->addWidget(speedGroup);

    /////////// style
    QGroupBox* styleGroup = new QGroupBox();

    QVBoxLayout* sliderlayout = new QVBoxLayout();

    for(int i = 0; i < 5; i++) {
        QHBoxLayout* sliderElemlayout = new QHBoxLayout();
        QCheckBox* styleBox = new QCheckBox();
        styleBox->setCheckable(true);
        styleBox->setProperty("i", i);

        connect(styleBox, SIGNAL(clicked()), mMotionWidget, SLOT(styleActivated())); 
        sliderElemlayout->addWidget(styleBox);

        QLabel* label;
        label = new QLabel(QString::fromStdString("style "+std::to_string(i)));
  
        sliderElemlayout->addWidget(label);

        QSlider* param = new QSlider(Qt::Horizontal);
        param->setMinimum(0);
        param->setMaximum(10);
        param->setSingleStep(1);
        param->setProperty("i", i);
        
        connect (param, SIGNAL(valueChanged(int)), mMotionWidget, SLOT(setStyle(const int&)));
     
        sliderElemlayout->addWidget(param);
        sliderlayout->addLayout(sliderElemlayout);
    }
    styleGroup->setLayout(sliderlayout);
    paramlayout->addWidget(styleGroup);
    
    //// timeline
    QGroupBox* timelineGroup = new QGroupBox();

    QVBoxLayout* timelineLayout = new QVBoxLayout();
    
    QHBoxLayout* timelineElemlayout = new QHBoxLayout();
    label = new QLabel(QString::fromStdString("timeline"));
    timelineElemlayout->addWidget(label);
    
    button = new QPushButton("current phase", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(setSliderCurrent())); 
    timelineElemlayout->addWidget(button);
    timelineLayout->addLayout(timelineElemlayout);

    std::vector<QSlider*> timelineWidgets;
    for(int i = 0; i < 5; i++) {
        QHBoxLayout* timelineElemlayout = new QHBoxLayout();
        QCheckBox* phaseBox = new QCheckBox();
        phaseBox->setCheckable(true);
        phaseBox->setProperty("i", i);

        connect(phaseBox, SIGNAL(clicked()), mMotionWidget, SLOT(phaseActivated())); 
        timelineElemlayout->addWidget(phaseBox);

        QLabel* label;
        label = new QLabel(QString::fromStdString("phase "+std::to_string(i)));
  
        timelineElemlayout->addWidget(label);

        QSlider* param = new QSlider(Qt::Horizontal);
        param->setMinimum(0);
        param->setMaximum(100);
        param->setSingleStep(1);
        param->setProperty("i", i);

        connect (param, SIGNAL(valueChanged(int)), mMotionWidget, SLOT(setPhase(const int&)));
     
        timelineElemlayout->addWidget(param);
        timelineWidgets.push_back(param);
        timelineLayout->addLayout(timelineElemlayout);
    }
    mMotionWidget->setTimeline(timelineWidgets);    
    timelineGroup->setLayout(timelineLayout);
    paramlayout->addWidget(timelineGroup);
    //////
    buttonlayout = new QHBoxLayout();


    button = new QPushButton("add", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(addConstraint())); 
    paramlayout->addWidget(button);


    mConstraintList = new QListWidget();
    mConstraintList->setMinimumSize(275,150);
    mConstraintList->setMaximumSize(275,150);
    
    paramlayout->addWidget(mConstraintList);

    paramlayout->addStretch(1);

    button = new QPushButton("print", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(printCurrentSettings())); 
    paramlayout->addWidget(button);

    mMainLayout->addLayout(paramlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
}
void 
AssemblerMainWindow::
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
AssemblerMainWindow::
onItemDoubleClicked(QListWidgetItem* _item)
{
    int idx = mDataList->row(_item);
    mConstraintList->clear();
    mMotionWidget->setRenderData(idx);
}
void 
AssemblerMainWindow::
addConstraint()
{
    std::string str = mMotionWidget->addConstraint();
    mConstraintList->addItem(QString::fromStdString(str));
}