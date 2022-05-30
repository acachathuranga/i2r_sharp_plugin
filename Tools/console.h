#ifndef CONSOLE_H
#define CONSOLE_H

#include <QObject>
#include <QWidget>
#include <QTextEdit>
#include <iostream>
#include <QScrollBar>
#include <QFile>
#include <QTextStream>
#include <QDateTime>

class Console : public QObject
{
    Q_OBJECT

    public:
        Console(QTextEdit *text_edit);
        Console(QTextEdit *text_edit, bool std_cout);
        ~Console();

        void print(std::string msg);
        void clear(void);

    signals:
        void printer_msg(QString msg);

    private slots:
        void print_msg(QString msg);

    private:
        // Private Attributes
        QTextEdit *text_edit_;
        bool cout = true;
        bool logToFile = true;
        QString logFileName = "iCube_SHARP_Log";
        QDateTime dateTime;
};

#endif // CONSOLE_H
