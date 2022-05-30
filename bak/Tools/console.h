#ifndef CONSOLE_H
#define CONSOLE_H

#include <QObject>
#include <QWidget>
#include <QTextEdit>
#include <iostream>
#include <QScrollBar>

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
};

#endif // CONSOLE_H
