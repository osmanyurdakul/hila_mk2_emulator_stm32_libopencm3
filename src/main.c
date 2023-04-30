#include "hila.h"
#include <stdbool.h>

extern volatile bool hila_cmd_isReceived;
extern volatile bool diag_cmd_isReceived;
extern volatile bool test_cmd_isReceived;

int main(void)
{
    hila_initialize();
    
    while(1)
    {

        if (hila_cmd_isReceived == true)
        {
            hila_rs485_cmd_execute();
            hila_cmd_isReceived = false;
        }

        if (test_cmd_isReceived == true)
        {
            hila_test_cmd_execute();
            test_cmd_isReceived = false;
        }
    }
}