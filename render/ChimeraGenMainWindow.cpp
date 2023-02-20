#include <iostream>
#include <QtWidgets/QApplication>
#include <QFormLayout>
#include <QGLWidget>
#include <QLabel>
#include <QCheckBox>
#include <QFileDialog>
#include "ChimeraGenMainWindow.h"

ChimeraGenMainWindow::
ChimeraGenMainWindow() :QMainWindow()
{
    setWindowTitle("Chimera generator");
}
ChimeraGenMainWindow::
ChimeraGenMainWindow(std::vector<RenderData> _renderData)
{   
    ChimeraGenMainWindow();
    mRenderData = _renderData;
    mDataIdx = 0;

    initLayoutSetting();
}
void
ChimeraGenMainWindow::
initLayoutSetting() {
    mMainLayout = new QHBoxLayout();
    setMaximumSize(1900,950);
    setMinimumSize(1900,950);

    QVBoxLayout *motionlayout = new QVBoxLayout();

    mMotionWidget = new ChimeraGenWidget(mRenderData);
    mMotionWidget->setMinimumSize(1600,900);
    mMotionWidget->setMaximumSize(1600,900);

    motionlayout->addWidget(mMotionWidget);

    QHBoxLayout *buttonlayout = new QHBoxLayout();

    QPushButton* button = new QPushButton("new", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(clearAll())); 
    buttonlayout->addWidget(button);
    buttonlayout->addStretch(1);

    button = new QPushButton("reset", this);
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

    ////////// data widget //////////////
    mDataList = new QListWidget();
    mDataList->setMinimumSize(275,200);
    mDataList->setMaximumSize(275,200);

    std::vector<std::string> filenames = RenderConfigParser::getFilenames(mRenderData);

    for(int i = 0; i < filenames.size(); i++) {
        mDataList->addItem(QString::fromStdString("config "+std::to_string(i)));
    }
    connect(mDataList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(changeData(QListWidgetItem*)));
    
    paramlayout->addWidget(mDataList);
    ////////// character generator layout /////////////////

    ////////// skel selecton widget 
    mSkelList = new QComboBox();
    std::vector<std::string> skelnames = RenderConfigParser::getSkelnames(mRenderData, 0);
    mMotionWidget->setSkelNames(skelnames);
    
    mSkelList->addItem(QString::fromStdString(""));
    for(int i = 0; i < skelnames.size(); i++) {
        mSkelList->addItem(QString::fromStdString(skelnames[i]));
    }
    connect(mSkelList, SIGNAL(currentTextChanged(const QString&)), this, SLOT(changeSkel(const QString&)));

    paramlayout->addWidget(mSkelList);

    ////////// skel viewer widget 
    mCharacterViewer = new CharacterViewer();
    mCharacterViewer->setMinimumSize(275,400);
    mCharacterViewer->setMaximumSize(275,400);
    paramlayout->addWidget(mCharacterViewer);
    connect(mCharacterViewer, SIGNAL(bodyClicked(int)), this, SLOT(bodyClicked(int)));

    ////////// body selection widget

    mBodyList = new QListWidget();
    mBodyList->setMinimumSize(275,200);
    mBodyList->setMaximumSize(275,200);
    connect(mBodyList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(bodyClicked(QListWidgetItem*)));
    
    paramlayout->addWidget(mBodyList);

    ////////// buton widget 
    QHBoxLayout* blockButtonlayout = new QHBoxLayout();

    button = new QPushButton("add block", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(addBodyBlock())); 
    blockButtonlayout->addWidget(button);

    button = new QPushButton("undo", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(popBodyBlock())); 
    blockButtonlayout->addWidget(button);
    paramlayout->addLayout(blockButtonlayout);

    QHBoxLayout* skelButtonlayout = new QHBoxLayout();
    button = new QPushButton("fix", this);
    connect(button, SIGNAL(clicked(bool)), mMotionWidget, SLOT(fixSkeleton())); 
    skelButtonlayout->addWidget(button);
    
    button = new QPushButton("import", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(importSkeleton())); 
    skelButtonlayout->addWidget(button);

    button = new QPushButton("export", this);
    connect(button, SIGNAL(clicked(bool)), this, SLOT(exportSkeleton())); 
    skelButtonlayout->addWidget(button);
    paramlayout->addLayout(skelButtonlayout);

    mMainLayout->addLayout(paramlayout);

    setCentralWidget(new QWidget());
    centralWidget()->setLayout(mMainLayout);
}
void 
ChimeraGenMainWindow::
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
ChimeraGenMainWindow::
changeData(QListWidgetItem* _item)
{
    mDataIdx = mDataList->row(_item);
    std::vector<std::string> skelnames = RenderConfigParser::getSkelnames(mRenderData, mDataIdx);
    mMotionWidget->setSkelNames(skelnames);

    mSkelList->clear();

    mSkelList->addItem(QString::fromStdString(""));
    for(int i = 0; i < skelnames.size(); i++) {
        mSkelList->addItem(QString::fromStdString(skelnames[i]));
    }   
    mMotionWidget->setRenderData(mDataIdx);

    mMotionWidget->setAssembleMode(true);
    mMotionWidget->clearBodyBlock();
    
    mBodyList->clear();
    mBodyChecked.clear();
}
void 
ChimeraGenMainWindow::
changeSkel(const QString& _str)
{
    if(_str.toStdString() != "") {
        mBodyList->clear();
        mBodyChecked.clear();

        int idx = mSkelList->currentIndex() - 1;
        mCharacterViewer->setSkeleton(_str.toStdString(), mRenderData[mDataIdx].CMpair[idx].second);

        std::vector<std::string> bodyNames = mCharacterViewer->getBodyNames();
        for(int i = 0; i < bodyNames.size(); i++) {
            mBodyList->addItem(QString::fromStdString(bodyNames[i]));
            mBodyChecked.push_back(false);
        }

    } else {
        mCharacterViewer->clearSkeleton();
        mBodyList->clear();
        mBodyChecked.clear();
    }
}
void 
ChimeraGenMainWindow::
bodyClicked(QListWidgetItem* _item)
{
    int idx = mBodyList->row(_item);
    mBodyChecked[idx] = !mBodyChecked[idx];

    if(mBodyChecked[idx]) {
        _item->setBackground(Qt::gray);
    } else {
        _item->setBackground(Qt::white);
    }
    mCharacterViewer->setBodyChecked(idx, mBodyChecked[idx]);
}
void 
ChimeraGenMainWindow::
bodyClicked(int _idx)
{
    mBodyChecked[_idx] = !mBodyChecked[_idx];
    QListWidgetItem* item = mBodyList->item(_idx);

    if(mBodyChecked[_idx]) {
        item->setBackground(Qt::gray);
    } else {
        item->setBackground(Qt::white);
    }
    mCharacterViewer->setBodyChecked(_idx, mBodyChecked[_idx]);
}
void
ChimeraGenMainWindow::
addBodyBlock()
{
    std::vector<std::string> bodyBlock;
    std::vector<std::string> bodyNames = mCharacterViewer->getBodyNames();

    for(int i = 0; i < mBodyChecked.size(); i++) {
        if(mBodyChecked[i]) {
            bodyBlock.push_back(bodyNames[i]);
            bodyClicked(i);
        }
    }
    if(bodyBlock.size() != 0) {
        int idx = mSkelList->currentIndex() - 1;
        std::string pose = mRenderData[mDataIdx].CMpair[idx].second;
        mMotionWidget->addBodyBlock(idx, mSkelList->currentText().toStdString(), bodyBlock);
    }
}
void
ChimeraGenMainWindow::
importSkeleton()
{
    QString fileName = QFileDialog::getOpenFileName(this, "open file", 
                        QString::fromStdString(std::string(SSC_DIR) + std::string("/data/character/assembled/")),"Files (*.*)");
    if(fileName.toStdString() == "")
        return;

    std::vector<std::string> skelnames = RenderConfigParser::getSkelnames(mRenderData, mDataIdx);
    mMotionWidget->setSkelNames(skelnames);
    for(int i = 0 ; i < skelnames.size(); i++) {
        skelnames[i] = std::string(SSC_DIR) + std::string("/data/character/") + skelnames[i];
    }
    std::pair<SIM::SkelInfo, std::map<int, Eigen::Vector3d>> result = SIM::SkeletonBuilder::assembleFromFile(fileName.toStdString(), skelnames);
    mMotionWidget->setAssembledSkeleton(result);
    std::cout << "import skeleton from " << fileName.toStdString() << std::endl;
}
void
ChimeraGenMainWindow::
exportSkeleton()
{
    QString fileName = QFileDialog::getSaveFileName(this, "save file", 
                        QString::fromStdString(std::string(SSC_DIR) + std::string("/data/character/assembled/")), "Files (*.*)");  
    if(fileName.toStdString() == "")
        return;

    SIM::SkelInfo recordSkel = mMotionWidget->getAssembledSkeleton();
    std::map<int, Eigen::Vector3d> map = mMotionWidget->getRotationMap();
    SIM::SkeletonBuilder::recordSkelInfoToFile(fileName.toStdString(), recordSkel, map);
    std::cout << "export skeleton to "  << fileName.toStdString() << std::endl;

}
void
ChimeraGenMainWindow::
clearAll()
{
    mBodyList->clear();
    mBodyChecked.clear();

    mMotionWidget->clearAll();
    mCharacterViewer->clearAll();
}