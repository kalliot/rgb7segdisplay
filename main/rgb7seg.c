#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "rgb7seg.h"


typedef struct color segment[2];

struct a7seg
{
    segment a;
    segment b;
    segment c;
    segment d;
    segment e;
    segment f;
    segment g;
};


#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define STRIP_LED_NUMBERS         58


static uint8_t led_strip_pixels[STRIP_LED_NUMBERS * 3];
static struct a7seg *display = (struct a7seg *) led_strip_pixels;


static rmt_channel_handle_t led_chan = NULL;
static rmt_tx_channel_config_t tx_chan_config = 
{
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = CONFIG_LEDSTRIP_GPIO,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
};
static rmt_encoder_handle_t led_encoder = NULL;
static led_strip_encoder_config_t encoder_config = 
{
    .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
};

static rmt_transmit_config_t tx_config = 
{
   .loop_count = 0, // no transfer loop
};



#define SET_SEGMENT(num,seg,color)  display[num].seg[0]=display[num].seg[1]=color


static void set_7seg(char *str, struct color c)
{
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    for (int i=0; i<strlen(str);i++)
    {
        if (i==4) break;

        switch (str[i])
        {
            case '1':
            case 'I':
            case 'i':
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
            break;

            case '2':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,d,c);
            break;
                
            case '3':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,d,c);
            break;

            case '4':
                SET_SEGMENT(i,f,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,c,c);
            break;

            case '5':
            case 's':
            case 'S':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,f,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,d,c);
            break;

            case '6':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,f,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,d,c);
            break;

            case '7':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
            break;

            case '8':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,d,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
                SET_SEGMENT(i,g,c);
            break;

            case '9':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,d,c);
                SET_SEGMENT(i,f,c);
                SET_SEGMENT(i,g,c);
            break;

            case '0':
            case 'O':
            case 'o':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,d,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case 'q':
            case 'Q':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,f,c);
                SET_SEGMENT(i,g,c);
            break;

            case 'A':
            case 'a':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case '-':
                SET_SEGMENT(i,g,c);
            break;

            case 'E':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,d,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case 'p':
            case 'P':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,b,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case 'f':
            case 'F':
                SET_SEGMENT(i,a,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case 't':
            case 'T':
                SET_SEGMENT(i,d,c);
                SET_SEGMENT(i,g,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case 'l':
            case 'L':
                SET_SEGMENT(i,d,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,f,c);
            break;

            case 'n':
            case 'N':
                SET_SEGMENT(i,c,c);
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,g,c);
            break;

            case 'r':
            case 'R':
                SET_SEGMENT(i,e,c);
                SET_SEGMENT(i,g,c);
            break;

            default:
            break;    
        }
    }
}

void rgb7seg_display(char *buff, struct color c)
{
    set_7seg(buff,c);
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));

}
void rgb7seg_init(void)
{
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}