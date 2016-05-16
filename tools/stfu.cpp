#include "motor.hpp"

int main(void)
{
    Motors *motors;
    motors = new Motors();
    motors->arm();

    return 0;
}
