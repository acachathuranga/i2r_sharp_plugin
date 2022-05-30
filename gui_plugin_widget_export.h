/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#ifndef EXPORT_WIDGET_H
#define EXPORT_WIDGET_H

// see https://wiki.qt.io/How_to_create_a_library_with_Qt_and_use_it_in_an_application
#include "gui_plugin_widget_global.h"
#include <QWidget>

// exported functions for main application to read data from this widget
extern "C" GUI_PLUGIN_WIDGET_EXPORT QWidget* createWidget();
extern "C" GUI_PLUGIN_WIDGET_EXPORT QString getVersion();
extern "C" GUI_PLUGIN_WIDGET_EXPORT void setRobotName(const QString &robot_name);
extern "C" GUI_PLUGIN_WIDGET_EXPORT void setUID(const int &uid);

#endif // EXPORT_WIDGET_H
