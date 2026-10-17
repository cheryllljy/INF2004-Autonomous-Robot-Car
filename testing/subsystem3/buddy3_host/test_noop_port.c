#include "b3_port.h"
ER b3_port_gpio_output_init(unsigned int pin, bool initial_high)
{
    (void)pin; (void)initial_high; return E_OK;
}
void b3_port_gpio_put(unsigned int pin, bool value)
{
    (void)pin; (void)value;
}
