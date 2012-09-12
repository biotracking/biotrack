/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Jun 21 12:42:55 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>
#include <ascribblearea.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionNew_Project;
    QAction *actionSave_Project;
    QAction *actionSave_Project_as;
    QAction *actionOpen_Project;
    QAction *actionSave_Template;
    QWidget *centralwidget;
    QPushButton *previousFrameButton;
    QSlider *currentTimeBar;
    QLabel *Framelabel;
    QPushButton *stopButton;
    QPushButton *nextFrameButton;
    QPushButton *targetpaint;
    QPushButton *playButton;
    QPushButton *nottargetpaint;
    QSpinBox *FramespinBox;
    QPushButton *saveFrameButton;
    QPushButton *clearButton;
    QLineEdit *targetIDbox;
    QLabel *targetIDLabel;
    QPushButton *backgroundmaskbutton;
    QScrollArea *imagescrollArea;
    QWidget *scrollAreaWidgetContents;
    QLabel *imageLabel;
    aScribbleArea *paintwidget;
    QPushButton *zoomButton;
    QComboBox *comboBox;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuSettings;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1056, 719);
        actionNew_Project = new QAction(MainWindow);
        actionNew_Project->setObjectName(QString::fromUtf8("actionNew_Project"));
        actionSave_Project = new QAction(MainWindow);
        actionSave_Project->setObjectName(QString::fromUtf8("actionSave_Project"));
        actionSave_Project_as = new QAction(MainWindow);
        actionSave_Project_as->setObjectName(QString::fromUtf8("actionSave_Project_as"));
        actionOpen_Project = new QAction(MainWindow);
        actionOpen_Project->setObjectName(QString::fromUtf8("actionOpen_Project"));
        actionSave_Template = new QAction(MainWindow);
        actionSave_Template->setObjectName(QString::fromUtf8("actionSave_Template"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        previousFrameButton = new QPushButton(centralwidget);
        previousFrameButton->setObjectName(QString::fromUtf8("previousFrameButton"));
        previousFrameButton->setGeometry(QRect(20, 540, 98, 27));
        currentTimeBar = new QSlider(centralwidget);
        currentTimeBar->setObjectName(QString::fromUtf8("currentTimeBar"));
        currentTimeBar->setGeometry(QRect(20, 510, 661, 29));
        currentTimeBar->setOrientation(Qt::Horizontal);
        Framelabel = new QLabel(centralwidget);
        Framelabel->setObjectName(QString::fromUtf8("Framelabel"));
        Framelabel->setGeometry(QRect(550, 540, 67, 17));
        stopButton = new QPushButton(centralwidget);
        stopButton->setObjectName(QString::fromUtf8("stopButton"));
        stopButton->setGeometry(QRect(230, 540, 98, 27));
        nextFrameButton = new QPushButton(centralwidget);
        nextFrameButton->setObjectName(QString::fromUtf8("nextFrameButton"));
        nextFrameButton->setGeometry(QRect(340, 540, 98, 27));
        targetpaint = new QPushButton(centralwidget);
        targetpaint->setObjectName(QString::fromUtf8("targetpaint"));
        targetpaint->setGeometry(QRect(750, 20, 98, 27));
        targetpaint->setCheckable(true);
        targetpaint->setChecked(true);
        targetpaint->setFlat(false);
        playButton = new QPushButton(centralwidget);
        playButton->setObjectName(QString::fromUtf8("playButton"));
        playButton->setGeometry(QRect(130, 540, 98, 27));
        nottargetpaint = new QPushButton(centralwidget);
        nottargetpaint->setObjectName(QString::fromUtf8("nottargetpaint"));
        nottargetpaint->setGeometry(QRect(750, 60, 98, 27));
        nottargetpaint->setCheckable(true);
        FramespinBox = new QSpinBox(centralwidget);
        FramespinBox->setObjectName(QString::fromUtf8("FramespinBox"));
        FramespinBox->setGeometry(QRect(620, 540, 59, 27));
        saveFrameButton = new QPushButton(centralwidget);
        saveFrameButton->setObjectName(QString::fromUtf8("saveFrameButton"));
        saveFrameButton->setGeometry(QRect(20, 580, 97, 27));
        clearButton = new QPushButton(centralwidget);
        clearButton->setObjectName(QString::fromUtf8("clearButton"));
        clearButton->setGeometry(QRect(740, 250, 98, 27));
        targetIDbox = new QLineEdit(centralwidget);
        targetIDbox->setObjectName(QString::fromUtf8("targetIDbox"));
        targetIDbox->setGeometry(QRect(750, 200, 113, 27));
        targetIDLabel = new QLabel(centralwidget);
        targetIDLabel->setObjectName(QString::fromUtf8("targetIDLabel"));
        targetIDLabel->setGeometry(QRect(750, 180, 101, 17));
        backgroundmaskbutton = new QPushButton(centralwidget);
        backgroundmaskbutton->setObjectName(QString::fromUtf8("backgroundmaskbutton"));
        backgroundmaskbutton->setGeometry(QRect(750, 100, 131, 27));
        backgroundmaskbutton->setCheckable(true);
        imagescrollArea = new QScrollArea(centralwidget);
        imagescrollArea->setObjectName(QString::fromUtf8("imagescrollArea"));
        imagescrollArea->setGeometry(QRect(10, 10, 721, 391));
        imagescrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        imagescrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        imagescrollArea->setWidgetResizable(false);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 706, 376));
        imageLabel = new QLabel(scrollAreaWidgetContents);
        imageLabel->setObjectName(QString::fromUtf8("imageLabel"));
        imageLabel->setGeometry(QRect(0, 0, 690, 360));
        imageLabel->setAutoFillBackground(false);
        imageLabel->setStyleSheet(QString::fromUtf8("font: 11pt \"Ubuntu\";\n"
"background-color: rgb(170, 0, 255);"));
        paintwidget = new aScribbleArea(scrollAreaWidgetContents);
        paintwidget->setObjectName(QString::fromUtf8("paintwidget"));
        paintwidget->setGeometry(QRect(0, 0, 690, 360));
        paintwidget->setStyleSheet(QString::fromUtf8("background-color: rgba(0, 46, 255, 10);"));
        imagescrollArea->setWidget(scrollAreaWidgetContents);
        zoomButton = new QPushButton(centralwidget);
        zoomButton->setObjectName(QString::fromUtf8("zoomButton"));
        zoomButton->setGeometry(QRect(750, 330, 97, 27));
        comboBox = new QComboBox(centralwidget);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(280, 410, 121, 27));
        MainWindow->setCentralWidget(centralwidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1056, 25));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuSettings = new QMenu(menuBar);
        menuSettings->setObjectName(QString::fromUtf8("menuSettings"));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuSettings->menuAction());
        menuFile->addAction(actionNew_Project);
        menuFile->addAction(actionOpen_Project);
        menuFile->addSeparator();
        menuFile->addAction(actionSave_Project);
        menuFile->addAction(actionSave_Project_as);
        menuSettings->addAction(actionSave_Template);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionNew_Project->setText(QApplication::translate("MainWindow", "New Project", 0, QApplication::UnicodeUTF8));
        actionSave_Project->setText(QApplication::translate("MainWindow", "Save Project", 0, QApplication::UnicodeUTF8));
        actionSave_Project_as->setText(QApplication::translate("MainWindow", "Save Project as...", 0, QApplication::UnicodeUTF8));
        actionOpen_Project->setText(QApplication::translate("MainWindow", "Open Project", 0, QApplication::UnicodeUTF8));
        actionSave_Template->setText(QApplication::translate("MainWindow", "Save Template", 0, QApplication::UnicodeUTF8));
        previousFrameButton->setText(QApplication::translate("MainWindow", "PreviousFrame", 0, QApplication::UnicodeUTF8));
        Framelabel->setText(QApplication::translate("MainWindow", "Frames", 0, QApplication::UnicodeUTF8));
        stopButton->setText(QApplication::translate("MainWindow", "Stop", 0, QApplication::UnicodeUTF8));
        nextFrameButton->setText(QApplication::translate("MainWindow", "NextFrame", 0, QApplication::UnicodeUTF8));
        targetpaint->setText(QApplication::translate("MainWindow", "Target", 0, QApplication::UnicodeUTF8));
        playButton->setText(QApplication::translate("MainWindow", "Play", 0, QApplication::UnicodeUTF8));
        nottargetpaint->setText(QApplication::translate("MainWindow", "Background", 0, QApplication::UnicodeUTF8));
        saveFrameButton->setText(QApplication::translate("MainWindow", "Save Frame", 0, QApplication::UnicodeUTF8));
        clearButton->setText(QApplication::translate("MainWindow", "Clear", 0, QApplication::UnicodeUTF8));
        targetIDbox->setText(QApplication::translate("MainWindow", "aTest", 0, QApplication::UnicodeUTF8));
        targetIDLabel->setText(QApplication::translate("MainWindow", "Target ID", 0, QApplication::UnicodeUTF8));
        backgroundmaskbutton->setText(QApplication::translate("MainWindow", "Background Mask", 0, QApplication::UnicodeUTF8));
        imageLabel->setText(QString());
        zoomButton->setText(QApplication::translate("MainWindow", "Zoom", 0, QApplication::UnicodeUTF8));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Fit to Window", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "100%", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "50%", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "200%", 0, QApplication::UnicodeUTF8)
        );
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuSettings->setTitle(QApplication::translate("MainWindow", "Settings", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
