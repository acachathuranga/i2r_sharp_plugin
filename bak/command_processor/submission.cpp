#include "submission.h"

SubMission::SubMission()
{

}

SubMission::SubMission(boost::function<bool ()> task)
{
    task_ = task;
}

SubMission::SubMission(boost::function<bool ()> task, boost::function<bool ()> fallback_task)
{
    task_ = task;
    fallback_tasks_.append(fallback_task);
}

SubMission::SubMission(boost::function<bool ()> task, boost::function<bool ()> fallback_task, int retry_limit)
{
    task_ = task;
    fallback_tasks_.append(fallback_task);
    retry_limit_ = retry_limit;
}


SubMission::SubMission(boost::function<bool ()> task, QVector<boost::function<bool ()>> fallback_tasks)
{
    task_ = task;
    fallback_tasks_ = fallback_tasks;
}

SubMission::SubMission(boost::function<bool ()> task, QVector<boost::function<bool ()>> fallback_tasks, int retry_limit)
{
    task_ = task;
    fallback_tasks_ = fallback_tasks;
    retry_limit_ = retry_limit;
}

void SubMission::addFallbackTask(boost::function<bool ()> fallback_task)
{
    fallback_tasks_.append(fallback_task);
}

void SubMission::setRetryLimit(int retry_limit)
{
    retry_limit_ = retry_limit;
}

void SubMission::setDescription(std::string desc)
{
    description = desc;
}

bool SubMission::isFallbackAllowed(void)
{
    return fallback_allowed_;
}

void SubMission::fallbackAllowed(bool status)
{
    fallback_allowed_ = status;
}
