#ifndef __RGB7SEG__
#define __RGB7SEG__


struct color 
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void rgb7seg_init(void);
void rgb7seg_display(char *buff, struct color c);

#endif