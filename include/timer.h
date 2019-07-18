#include <iostream>
#include <chrono>


class Timer
{
public:
    Timer(std::chrono::seconds timerDuration)
            : mStart(std::chrono::system_clock::now()) ,
              mTimerDuration(timerDuration) { }

    void reset()
    {
        mStart = std::chrono::system_clock::now();
        ROS_INFO_STREAM_THROTTLE(10,"Clock Reset. Tripos is Ok");
    }

    bool isEnded() const
    {
        return elapsedSec() > mTimerDuration.count();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> mStart;
    std::chrono::seconds mTimerDuration;
    double elapsedSec() const
    {
        return std::chrono::duration_cast<std::chrono::seconds>
                (std::chrono::system_clock::now() - mStart).count();
    }
};