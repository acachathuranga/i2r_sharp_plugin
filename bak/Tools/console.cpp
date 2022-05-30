#include "console.h"

Console::Console(QTextEdit *text_edit)
{
    text_edit_ = text_edit;
    text_edit_->setTextColor(QColor(0, 255, 0));
    text_edit_->setReadOnly(true);

    QObject::connect(this, &Console::printer_msg, this, &Console::print_msg);
}

Console::Console(QTextEdit *text_edit, bool std_cout)
{
    text_edit_ = text_edit;
    text_edit_->setTextColor(QColor(0, 255, 0));
    text_edit_->setReadOnly(true);

    cout = std_cout;

    QObject::connect(this, &Console::printer_msg, this, &Console::print_msg);
}

Console::~Console()
{
    // Nothing to clean
}

void Console::print_msg(QString msg)
{
    text_edit_->append(msg);
    QScrollBar *sb = text_edit_->verticalScrollBar();
    sb->setValue(sb->maximum());
    if (cout)
    {
        std::cout << msg.toStdString() << std::endl;
    }
}

void Console::print(std::string msg)
{
    emit(printer_msg(QString(msg.c_str())));
}

void Console::clear()
{
    text_edit_->clear();
}
