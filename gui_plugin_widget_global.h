/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#ifndef GUI_PLUGIN_WIDGET_GLOBAL_H
#define GUI_PLUGIN_WIDGET_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(GUI_PLUGIN_WIDGET_LIBRARY)
#define GUI_PLUGIN_WIDGET_EXPORT Q_DECL_EXPORT
#else
#define GUI_PLUGIN_WIDGET_EXPORT Q_DECL_IMPORT
#endif

#endif // GUI_PLUGIN_WIDGET_GLOBAL_H
