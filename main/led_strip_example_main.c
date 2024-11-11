#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rgb7seg.h"

#define CHASE_SPEED_MS   100

void app_main(void)
{
    rgb7seg_init();

    while (1) 
    {
        struct color c = { 10,0,0};
        char buff[6];
        for (int i = 0; i < 300; i++)
        {        
            if (i < 100) c.r = 20;
            if (i > 100) { c.g = 20; c.r = 0; }
            if (i > 200) { c.b = 20; c.g = 0; }
            sprintf(buff,"%4d",i);
            printf("%s\n",buff);
            rgb7seg_display(buff,c);
            vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
        }    
    }
}