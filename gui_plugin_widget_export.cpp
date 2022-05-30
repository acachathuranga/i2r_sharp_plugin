/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#include "gui_plugin_widget_export.h"
#include "plugin_template.h"
#include <QDebug>
#include <QString>
#include <QVariant>

QWidget *wid = NULL;

/**
 * Creates this widget and returns the handle to the widget
 * @return Handle to widget
 */
QWidget *createWidget()
{
    wid = new gui_plugin::SHARP();
    return wid;
}

/**
 * Returns the version of this library
 * @return String version representation of this library
 */
QString getVersion()
{
    return PLUGIN_VERSION;
}

/**
 * Sets the robot name
 * @param robot_name Name of robot
 */
void setRobotName(const QString &robot_name)
{
    if (wid != NULL)
    {
        gui_plugin::BaseWidget *basew = dynamic_cast<gui_plugin::BaseWidget *>(wid);
        if (basew)
        {
            basew->setRobotName(robot_name);
        }
    }
}

/**
 * Assigns a UID to this library
 * @param uid ID assigned to this library
 */
void setUID(const int &uid)
{
    if (wid != NULL)
    {
        wid->setProperty(QString(K_PROPERTY_NAME_WIDGET_UID).toStdString().c_str(), uid);
        qDebug() << PLUGIN_NAME << "uid" << wid->property(QString(K_PROPERTY_NAME_WIDGET_UID).toStdString().c_str()).toInt();
    }
}
