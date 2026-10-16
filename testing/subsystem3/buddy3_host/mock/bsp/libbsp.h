#ifndef MOCK_BSP_LIBBSP_H
#define MOCK_BSP_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_OUT 1U
#define GPIO_MODE_IN 0U
ER gpio_set_pin(UINT no, UINT mode);
ER gpio_set_val(UINT no, UINT val);
UINT gpio_get_val(UINT no);
#endif
