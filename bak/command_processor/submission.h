#ifndef SUBMISSION_H
#define SUBMISSION_H

#include <QObject>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <QVector>

/**
 * @brief The SubMission class
 *
 * A SubMission has only one task.
 * A SubMission may or may not have one or more fallback tasks.
 * Fallback tasks will be executed when a cancellation request has been issued
 * Default retry_limit for a task is 1.
 */
class SubMission
{

public:
    SubMission();
    SubMission(boost::function<bool ()> task);
    SubMission(boost::function<bool ()> task, boost::function<bool ()> fallback_task);
    SubMission(boost::function<bool ()> task, boost::function<bool ()> fallback_task, int retry_limit);
    SubMission(boost::function<bool ()> task, QVector<boost::function<bool ()>> fallback_tasks);
    SubMission(boost::function<bool ()> task, QVector<boost::function<bool ()>> fallback_tasks, int retry_limit);

    void addFallbackTask(boost::function<bool ()> fallback_task);
    void setRetryLimit(int retry_limit);
    void setDescription(std::string desc);
    bool isFallbackAllowed(void);
    void fallbackAllowed(bool status);

    std::string description = "";

signals:

public slots:

private:
    boost::function<bool ()> task_;
    QVector<boost::function<bool ()>> fallback_tasks_;
    bool fallback_allowed_ = true;

    int retry_limit_ = 1;
};

#endif // SUBMISSION_H
