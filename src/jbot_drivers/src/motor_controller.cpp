#include "motor_controller.h"
#include <chrono>
#include <cppgpio.hpp>

using namespace GPIO;

int main()
{
    // use gpio #18

    DigitalOut motor1A(18);
    DigitalOut motor1B(12);

    // stop
    motor1A.off();
    motor1B.off();

    // fwd
    motor1A.off();
    motor1B.on();

    // wait some time

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // switch it off again

    motor1A.off();
    motor1B.off();

    return 0;
}