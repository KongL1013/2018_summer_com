/********************************************************************************
** Form generated from reading UI file 'DroneCompetition.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DRONECOMPETITION_H
#define UI_DRONECOMPETITION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DroneCompetitionClass
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget_PaintArea;
    QWidget *tab;
    QLabel *label;
    QLabel *label_Test1;
    QLabel *label_3;
    QLabel *label_5;
    QLabel *label_Test2;
    QLabel *label_Test3;
    QTextBrowser *textBrowser;
    QFrame *frame;
    QLabel *label_FrontImage;
    QLabel *label_2;
    QPushButton *pushButton_FrontImgRGB_Cap;
    QFrame *frame_2;
    QLabel *label_FrontImageDep;
    QLabel *label_4;
    QPushButton *pushButton_FrontImgDep_Cap;
    QFrame *frame_3;
    QLabel *label_DownImage;
    QLabel *label_7;
    QPushButton *pushButton_DownImgRGB_Cap;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *DroneCompetitionClass)
    {
        if (DroneCompetitionClass->objectName().isEmpty())
            DroneCompetitionClass->setObjectName(QStringLiteral("DroneCompetitionClass"));
        DroneCompetitionClass->resize(1127, 787);
        centralWidget = new QWidget(DroneCompetitionClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        tabWidget_PaintArea = new QTabWidget(centralWidget);
        tabWidget_PaintArea->setObjectName(QStringLiteral("tabWidget_PaintArea"));
        tabWidget_PaintArea->setGeometry(QRect(20, 40, 446, 290));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        label = new QLabel(tab);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 30, 54, 11));
        label_Test1 = new QLabel(tab);
        label_Test1->setObjectName(QStringLiteral("label_Test1"));
        label_Test1->setGeometry(QRect(70, 30, 54, 11));
        label_3 = new QLabel(tab);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 50, 54, 11));
        label_5 = new QLabel(tab);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 70, 54, 11));
        label_Test2 = new QLabel(tab);
        label_Test2->setObjectName(QStringLiteral("label_Test2"));
        label_Test2->setGeometry(QRect(70, 50, 54, 11));
        label_Test3 = new QLabel(tab);
        label_Test3->setObjectName(QStringLiteral("label_Test3"));
        label_Test3->setGeometry(QRect(70, 70, 54, 11));
        textBrowser = new QTextBrowser(tab);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(150, 10, 261, 231));
        tabWidget_PaintArea->addTab(tab, QString());
        frame = new QFrame(centralWidget);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(580, 20, 491, 321));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        label_FrontImage = new QLabel(frame);
        label_FrontImage->setObjectName(QStringLiteral("label_FrontImage"));
        label_FrontImage->setGeometry(QRect(20, 40, 461, 281));
        label_2 = new QLabel(frame);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 0, 54, 11));
        pushButton_FrontImgRGB_Cap = new QPushButton(frame);
        pushButton_FrontImgRGB_Cap->setObjectName(QStringLiteral("pushButton_FrontImgRGB_Cap"));
        pushButton_FrontImgRGB_Cap->setGeometry(QRect(360, 0, 75, 23));
        frame_2 = new QFrame(centralWidget);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(20, 370, 491, 321));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        label_FrontImageDep = new QLabel(frame_2);
        label_FrontImageDep->setObjectName(QStringLiteral("label_FrontImageDep"));
        label_FrontImageDep->setGeometry(QRect(20, 40, 461, 281));
        label_4 = new QLabel(frame_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 0, 54, 11));
        pushButton_FrontImgDep_Cap = new QPushButton(frame_2);
        pushButton_FrontImgDep_Cap->setObjectName(QStringLiteral("pushButton_FrontImgDep_Cap"));
        pushButton_FrontImgDep_Cap->setGeometry(QRect(360, 0, 75, 23));
        frame_3 = new QFrame(centralWidget);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(580, 370, 491, 321));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        label_DownImage = new QLabel(frame_3);
        label_DownImage->setObjectName(QStringLiteral("label_DownImage"));
        label_DownImage->setGeometry(QRect(20, 40, 461, 281));
        label_7 = new QLabel(frame_3);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 0, 54, 11));
        pushButton_DownImgRGB_Cap = new QPushButton(frame_3);
        pushButton_DownImgRGB_Cap->setObjectName(QStringLiteral("pushButton_DownImgRGB_Cap"));
        pushButton_DownImgRGB_Cap->setGeometry(QRect(360, 0, 75, 23));
        DroneCompetitionClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(DroneCompetitionClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1127, 23));
        DroneCompetitionClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(DroneCompetitionClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        DroneCompetitionClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(DroneCompetitionClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        DroneCompetitionClass->setStatusBar(statusBar);

        retranslateUi(DroneCompetitionClass);

        tabWidget_PaintArea->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(DroneCompetitionClass);
    } // setupUi

    void retranslateUi(QMainWindow *DroneCompetitionClass)
    {
        DroneCompetitionClass->setWindowTitle(QApplication::translate("DroneCompetitionClass", "DroneCompetition", Q_NULLPTR));
        label->setText(QApplication::translate("DroneCompetitionClass", "Test1:", Q_NULLPTR));
        label_Test1->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_3->setText(QApplication::translate("DroneCompetitionClass", "Test2:", Q_NULLPTR));
        label_5->setText(QApplication::translate("DroneCompetitionClass", "Test3:", Q_NULLPTR));
        label_Test2->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_Test3->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        tabWidget_PaintArea->setTabText(tabWidget_PaintArea->indexOf(tab), QApplication::translate("DroneCompetitionClass", "Tab 1", Q_NULLPTR));
        label_FrontImage->setText(QApplication::translate("DroneCompetitionClass", "Front Image RGB", Q_NULLPTR));
        label_2->setText(QApplication::translate("DroneCompetitionClass", "Front RGB", Q_NULLPTR));
        pushButton_FrontImgRGB_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture", Q_NULLPTR));
        label_FrontImageDep->setText(QApplication::translate("DroneCompetitionClass", "Front Image Depth", Q_NULLPTR));
        label_4->setText(QApplication::translate("DroneCompetitionClass", "Front DEP", Q_NULLPTR));
        pushButton_FrontImgDep_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture", Q_NULLPTR));
        label_DownImage->setText(QApplication::translate("DroneCompetitionClass", "Down Image RGB", Q_NULLPTR));
        label_7->setText(QApplication::translate("DroneCompetitionClass", "Down RGB", Q_NULLPTR));
        pushButton_DownImgRGB_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DroneCompetitionClass: public Ui_DroneCompetitionClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DRONECOMPETITION_H
