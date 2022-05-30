#-------------------------------------------------
#
# Project created by Achala Athukorala (achala_chathuranga@sutd.edu.sg)
#
#-------------------------------------------------

QT       += widgets

TARGET = sharp
TEMPLATE = lib
CONFIG += c++11
#LIBS += -ljsoncpp
LIBS += -lpaho-mqttpp3 -lpaho-mqtt3as -lyaml-cpp

#Setting Command Publishing Version
DEFINES += USING_COMMANDPUB2
CONFIG += use_commandpub2

DEFINES += PLUGIN_VERSION=\\\"0.0.1\\\"
DEFINES += PLUGIN_NAME=\\\"Demo\\\" #$$TARGET
DEFINES += GUI_PLUGIN_WIDGET_LIBRARY

# specify build directories
CONFIG(debug, debug|release) {
    # debug mode
    DESTDIR = ../../../bin/plugins/$$TARGET
}
else:CONFIG(force_debug_info) {
    # profile mode
    DESTDIR = ../../../bin/plugins/$$TARGET
}
else {
    # release mode
    DESTDIR = ../../../bin/plugins/$$TARGET
}
OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
UI_DIR = $$DESTDIR/.ui

INCLUDEPATH += ../common


SOURCES += plugin_template.cpp \
    gui_plugin_widget_export.cpp  \
    ../../common/gui/common_utils.cpp \
    ../../common/gui/websocketdata.cpp \
    ../../common/mission/basedata.cpp \
    ../../common/mission/mission_completed_data.cpp \
    command_processor/commandprocessor.cpp \
    Tools/console.cpp \
    Tools/robotCommunication.cpp

HEADERS += plugin_template.h \
    gui_plugin_widget_export.h \
    gui_plugin_widget_global.h \
    ../common/basewidget.h  \
    ../../common/gui/common_utils.h \
    ../../common/gui/websocketdata.h \
    ../../common/mission/basedata.h \
    ../../common/mission/mission_completed_data.h \
    ../../common/fsm_defs.h \
    command_processor/commandprocessor.h \
    Tools/console.h \
    Tools/robotCommunication.h


use_commandpub2 {
HEADERS +=  ../../common/mission/statuspub2_system_status_data.h \
    ../../common/mission/tasks_status_data.h \
    ../../common/mission/robot_status_data2.h \
    ../../common/mission/mission_data2.h \
    ../../common/mission/submission_data2.h

SOURCES +=  ../../common/mission/statuspub2_system_status_data.cpp \
    ../../common/mission/tasks_status_data.cpp \
    ../../common/mission/robot_status_data2.cpp \
    ../../common/mission/mission_data2.cpp  \
    ../../common/mission/submission_data2.cpp
} else {
HEADERS +=  ../../common/mission/statuspubdata.h \
    ../../common/mission/robot_status_data.h \
    ../../common/mission/mission_data.h \
    ../../common/mission/submission_data.h

SOURCES += ../../common/mission/statuspubdata.cpp \
    ../../common/mission/robot_status_data.cpp \
    ../../common/mission/mission_data.cpp \
    ../../common/mission/submission_data.cpp
}

FORMS += \
    plugin_template.ui

RESOURCES += \
    plugin_resources.qrc

# Copy Config file
#QMAKE_POST_LINK += yes | cp mission_config.yaml $$DESTDIR/../../


