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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
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
    QTextBrowser *textBrowser;
    QFrame *frame;
    QLabel *label_FrontImage;
    QLabel *label_2;
    QPushButton *pushButton_FrontImgRGB_Cap;
    QPushButton *pushButton_FrontImg_Cap;
    QFrame *frame_2;
    QLabel *label_FrontImageDep;
    QLabel *label_4;
    QPushButton *pushButton_FrontImgDep_Cap;
    QLineEdit *etSavePath;
    QCheckBox *cbIsFront;
    QFrame *frame_3;
    QLabel *label_DownImage;
    QLabel *label_7;
    QPushButton *pushButton_DownImgRGB_Cap;
    QLabel *label_Test2;
    QLabel *gyo_z;
    QLabel *mag_y;
    QLabel *acc_y;
    QLabel *label_10;
    QLabel *mag_z;
    QLabel *label_6;
    QLabel *label;
    QLabel *label_3;
    QLabel *label_Test3;
    QLabel *gps_e;
    QLabel *gyo_y;
    QLabel *label_Test1;
    QLabel *mag_x;
    QLabel *label_12;
    QLabel *label_9;
    QLabel *label_13;
    QLabel *gyo_x;
    QLabel *acc_z;
    QLabel *label_11;
    QLabel *acc_x;
    QLabel *gps_n;
    QLabel *label_14;
    QLabel *label_8;
    QLabel *label_5;
    QLabel *gps_h;
    QLabel *label_15;
    QLabel *baro;
    QLabel *label_16;
    QLabel *pz;
    QLabel *label_17;
    QLabel *py;
    QLabel *label_18;
    QLabel *label_19;
    QLabel *px;
    QLabel *vz;
    QLabel *vy;
    QLabel *vx;
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
        textBrowser = new QTextBrowser(tab);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(30, 30, 381, 201));
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
        pushButton_FrontImgRGB_Cap->setGeometry(QRect(320, 0, 75, 23));
        pushButton_FrontImg_Cap = new QPushButton(frame);
        pushButton_FrontImg_Cap->setObjectName(QStringLiteral("pushButton_FrontImg_Cap"));
        pushButton_FrontImg_Cap->setGeometry(QRect(410, 0, 75, 23));
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
        etSavePath = new QLineEdit(frame_2);
        etSavePath->setObjectName(QStringLiteral("etSavePath"));
        etSavePath->setGeometry(QRect(220, 0, 113, 20));
        cbIsFront = new QCheckBox(frame_2);
        cbIsFront->setObjectName(QStringLiteral("cbIsFront"));
        cbIsFront->setGeometry(QRect(140, 0, 71, 21));
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
        label_Test2 = new QLabel(centralWidget);
        label_Test2->setObjectName(QStringLiteral("label_Test2"));
        label_Test2->setGeometry(QRect(440, 340, 54, 11));
        gyo_z = new QLabel(centralWidget);
        gyo_z->setObjectName(QStringLiteral("gyo_z"));
        gyo_z->setGeometry(QRect(520, 310, 54, 11));
        mag_y = new QLabel(centralWidget);
        mag_y->setObjectName(QStringLiteral("mag_y"));
        mag_y->setGeometry(QRect(520, 170, 54, 11));
        acc_y = new QLabel(centralWidget);
        acc_y->setObjectName(QStringLiteral("acc_y"));
        acc_y->setGeometry(QRect(520, 230, 54, 11));
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(480, 230, 54, 11));
        mag_z = new QLabel(centralWidget);
        mag_z->setObjectName(QStringLiteral("mag_z"));
        mag_z->setGeometry(QRect(520, 190, 54, 11));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(480, 170, 54, 11));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(280, 0, 54, 11));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(280, 20, 54, 11));
        label_Test3 = new QLabel(centralWidget);
        label_Test3->setObjectName(QStringLiteral("label_Test3"));
        label_Test3->setGeometry(QRect(510, 340, 54, 11));
        gps_e = new QLabel(centralWidget);
        gps_e->setObjectName(QStringLiteral("gps_e"));
        gps_e->setGeometry(QRect(320, 20, 54, 11));
        gyo_y = new QLabel(centralWidget);
        gyo_y->setObjectName(QStringLiteral("gyo_y"));
        gyo_y->setGeometry(QRect(520, 290, 54, 11));
        label_Test1 = new QLabel(centralWidget);
        label_Test1->setObjectName(QStringLiteral("label_Test1"));
        label_Test1->setGeometry(QRect(370, 340, 54, 11));
        mag_x = new QLabel(centralWidget);
        mag_x->setObjectName(QStringLiteral("mag_x"));
        mag_x->setGeometry(QRect(520, 150, 54, 11));
        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(480, 270, 54, 11));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(480, 210, 54, 11));
        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(480, 290, 54, 11));
        gyo_x = new QLabel(centralWidget);
        gyo_x->setObjectName(QStringLiteral("gyo_x"));
        gyo_x->setGeometry(QRect(520, 270, 54, 11));
        acc_z = new QLabel(centralWidget);
        acc_z->setObjectName(QStringLiteral("acc_z"));
        acc_z->setGeometry(QRect(520, 250, 54, 11));
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(480, 250, 54, 11));
        acc_x = new QLabel(centralWidget);
        acc_x->setObjectName(QStringLiteral("acc_x"));
        acc_x->setGeometry(QRect(520, 210, 54, 11));
        gps_n = new QLabel(centralWidget);
        gps_n->setObjectName(QStringLiteral("gps_n"));
        gps_n->setGeometry(QRect(320, 0, 54, 11));
        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(480, 310, 54, 11));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(480, 190, 54, 11));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(480, 150, 54, 11));
        gps_h = new QLabel(centralWidget);
        gps_h->setObjectName(QStringLiteral("gps_h"));
        gps_h->setGeometry(QRect(520, 110, 54, 11));
        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(480, 110, 54, 11));
        baro = new QLabel(centralWidget);
        baro->setObjectName(QStringLiteral("baro"));
        baro->setGeometry(QRect(520, 90, 54, 11));
        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(480, 90, 54, 11));
        pz = new QLabel(centralWidget);
        pz->setObjectName(QStringLiteral("pz"));
        pz->setGeometry(QRect(430, 40, 54, 11));
        label_17 = new QLabel(centralWidget);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(390, 40, 54, 11));
        py = new QLabel(centralWidget);
        py->setObjectName(QStringLiteral("py"));
        py->setGeometry(QRect(430, 20, 54, 11));
        label_18 = new QLabel(centralWidget);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(390, 0, 54, 11));
        label_19 = new QLabel(centralWidget);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(390, 20, 54, 11));
        px = new QLabel(centralWidget);
        px->setObjectName(QStringLiteral("px"));
        px->setGeometry(QRect(430, 0, 54, 11));
        vz = new QLabel(centralWidget);
        vz->setObjectName(QStringLiteral("vz"));
        vz->setGeometry(QRect(520, 40, 54, 11));
        vy = new QLabel(centralWidget);
        vy->setObjectName(QStringLiteral("vy"));
        vy->setGeometry(QRect(520, 20, 54, 11));
        vx = new QLabel(centralWidget);
        vx->setObjectName(QStringLiteral("vx"));
        vx->setGeometry(QRect(520, 0, 54, 11));
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
        tabWidget_PaintArea->setTabText(tabWidget_PaintArea->indexOf(tab), QApplication::translate("DroneCompetitionClass", "Tab 1", Q_NULLPTR));
        label_FrontImage->setText(QApplication::translate("DroneCompetitionClass", "Front Image RGB", Q_NULLPTR));
        label_2->setText(QApplication::translate("DroneCompetitionClass", "Front RGB", Q_NULLPTR));
        pushButton_FrontImgRGB_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture", Q_NULLPTR));
        pushButton_FrontImg_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture 2", Q_NULLPTR));
        label_FrontImageDep->setText(QApplication::translate("DroneCompetitionClass", "Front Image Depth", Q_NULLPTR));
        label_4->setText(QApplication::translate("DroneCompetitionClass", "Front DEP", Q_NULLPTR));
        pushButton_FrontImgDep_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture", Q_NULLPTR));
        cbIsFront->setText(QApplication::translate("DroneCompetitionClass", "front", Q_NULLPTR));
        label_DownImage->setText(QApplication::translate("DroneCompetitionClass", "Down Image RGB", Q_NULLPTR));
        label_7->setText(QApplication::translate("DroneCompetitionClass", "Down RGB", Q_NULLPTR));
        pushButton_DownImgRGB_Cap->setText(QApplication::translate("DroneCompetitionClass", "Capture", Q_NULLPTR));
        label_Test2->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        gyo_z->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        mag_y->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        acc_y->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_10->setText(QApplication::translate("DroneCompetitionClass", "acc_y", Q_NULLPTR));
        mag_z->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_6->setText(QApplication::translate("DroneCompetitionClass", "mag_y", Q_NULLPTR));
        label->setText(QApplication::translate("DroneCompetitionClass", "GPS N", Q_NULLPTR));
        label_3->setText(QApplication::translate("DroneCompetitionClass", "GPS E", Q_NULLPTR));
        label_Test3->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        gps_e->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        gyo_y->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_Test1->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        mag_x->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_12->setText(QApplication::translate("DroneCompetitionClass", "gyo_x", Q_NULLPTR));
        label_9->setText(QApplication::translate("DroneCompetitionClass", "acc_x", Q_NULLPTR));
        label_13->setText(QApplication::translate("DroneCompetitionClass", "gyo_y", Q_NULLPTR));
        gyo_x->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        acc_z->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_11->setText(QApplication::translate("DroneCompetitionClass", "acc_z", Q_NULLPTR));
        acc_x->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        gps_n->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_14->setText(QApplication::translate("DroneCompetitionClass", "gyo_z", Q_NULLPTR));
        label_8->setText(QApplication::translate("DroneCompetitionClass", "mag_z", Q_NULLPTR));
        label_5->setText(QApplication::translate("DroneCompetitionClass", "mag_x", Q_NULLPTR));
        gps_h->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_15->setText(QApplication::translate("DroneCompetitionClass", "GPS h", Q_NULLPTR));
        baro->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_16->setText(QApplication::translate("DroneCompetitionClass", "BARO", Q_NULLPTR));
        pz->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_17->setText(QApplication::translate("DroneCompetitionClass", "Z_EST", Q_NULLPTR));
        py->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        label_18->setText(QApplication::translate("DroneCompetitionClass", "X_EST", Q_NULLPTR));
        label_19->setText(QApplication::translate("DroneCompetitionClass", "Y_EST", Q_NULLPTR));
        px->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        vz->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        vy->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
        vx->setText(QApplication::translate("DroneCompetitionClass", "0", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DroneCompetitionClass: public Ui_DroneCompetitionClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DRONECOMPETITION_H
