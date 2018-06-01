#pragma once

//*******paintWiget.h*******

#ifndef PAINTWIGET_H
#define PAINTWIGET_H

#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTabWidget>

#define painting_area_width 446
#define painting_area_height 266

class StatusPainter :public QWidget
{
public:
StatusPainter();

double pitchd;
double rolld;
double pitch;
double roll;
double yaw;
double yawd;
double compassd;

protected:
void paintEvent(QPaintEvent *event);

private:
float arc_center_x;
float arc_center_y;
float arc_diameter;
QString roll_string;
QPointF ground_points[5];
QPointF arrow_points[4];
QRectF arc_rectangle;

void painting_Calculate();
};

#endif
