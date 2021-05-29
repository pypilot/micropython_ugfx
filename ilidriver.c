/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"


#define TFT_CMD_SWRESET		0x01
/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define TDISPLAY

#ifdef TDISPLAY
#define PIN_NUM_MISO 17
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_DC   16
#define PIN_NUM_RST  20
#define LINE_LEN 136

#else

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   27

#define PIN_NUM_DC   32
#define PIN_NUM_RST  05

#define LINE_LEN 240
#endif

#define PIN_NUM_BCKL 4

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 20 // must divide into 240 and 320

static spi_device_handle_t ili_spi;
static uint16_t *dma_lines[2];
static uint8_t backlight = 0;

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
 aa*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

#if 0
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    {0x36, {0x28}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};
#elif 1
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
//    {0x01, {0}, 0}, // software reset
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02},  5},
    {0xCF, {0x00, 0X83, 0X30},  3},
//    {0xEF, {0x03, 0x80, 0x02},  3},
    {0xE8, {0x84, 0x11, 0x7a},  3},
    {0xEA, {0x00, 0x00},  2},
    {0xED, {0x64, 0x03, 0X12, 0X81},  4},
    {0xF7, {0x30},  1},
    {0xC0, {0x29},  1},  //Power control VRH[5:0]
    {0xC1, {0x00},  1},  //Power control SAP[2:0];BT[3:0]
    {0xC5, {0x31, 0x08},  2},  //VCM control
    {0xC7, {0xC0},  1},  //VCM control2
#ifdef TDISPLAY
    {0x36, {0xC8}, 1},
    {0x21, {0}, 0}, // invert on
#else
    {0x36, {0x48}, 1},
    {0x20, {0}, 0}, // invert off
#endif 
    {0x3A, {0x55},  1},  // *** INTERFACE PIXEL FORMAT: 0x66 -> 18 bit; 0x55 -> 16 bit
     {0xB1, {0x00, 0x1B},  2},
     {0xB6, {0x08, 0x82, 0x27, 0x00},  4},  // Display Function Control
//    {0x30, {0x00, 0x00, 0x01, 0x3F},  4},
    {0xF2, {0x02},  1},  // 3Gamma Function: Disable (0x02), Enable (0x03)
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0xE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
//    {0x26, {0x08},  1},  //Gamma curve selected (0x01, 0x02, 0x04, 0x08)
//    {0xE0, {  	  0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00},  15},						//Positive Gamma Correction
//    {0xE1, { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F},  15},
    {0x11, {0}, 0x80},    //  Sleep out
     {0x29, {0}, 0x80},    // Display on
    {0, {0}, 0xff}
};
#else
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
//    {0x01, {0}, 0}, // software reset
     {0x20, {0}, 0}, // invert off
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02},  5},
    {0xCF, {0x00, 0xC1, 0x30},  3},
    {0xEF, {0x03, 0x80, 0x02},  3},
    {0xE8, {0x85, 0x00, 0x78},  3},
    {0xEA, {0x00, 0x00},  2},
    {0xED, {0x64, 0x03, 0x12, 0x81},  4},
    {0xF7, {0x20},  1},
    {0xC0, {0x23},  1},  //Power control VRH[5:0]
    {0xC1, {0x10},  1},  //Power control SAP[2:0];BT[3:0]
    {0xC5, {0x3e, 0x28},  2},  //VCM control
    {0xC7, {0x86},  1},  //VCM control2
#ifdef TDISPLAY
    {0x36, {0x28}, 1},
#else
    {0x36, {0x48}, 1},
#endif 
    {0x3A, {0x55},  1},  // *** INTERFACE PIXEL FORMAT: 0x66 -> 18 bit; 0x55 -> 16 bit
     {0xB1, {0x03, 0x1f},  2},
     {0xB6, {0x08, 0x82, 0x27, 0x00},  4},  // Display Function Control
//    {0x30, {0x00, 0x00, 0x01, 0x3F},  4},
//    {0xF2, {0x00},  1},  // 3Gamma Function: Disable (0x02), Enable (0x03)
//    {0x26, {0x01},  1},  //Gamma curve selected (0x01, 0x02, 0x04, 0x08)
//    {0xE0, {  	  0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00},  15},						//Positive Gamma Correction
//    {0xE1, { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F},  15},
    {0x11, {0}, 0x80},    //  Sleep out
     {0x29, {0}, 0x80},    // Display on
    {0, {0}, 0xff}
};
#endif

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
static void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(ili_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
static void lcd_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(ili_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
static void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

static uint32_t lcd_get_id()
{
    //get_id cmd
    lcd_cmd(0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=8*3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(ili_spi, &t);
    assert( ret == ESP_OK );

    uint32_t id;
    memcpy(&id, t.rx_data, 4);
    return id;
}

//Initialize the display
static void lcd_init()
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;


    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(50 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(50 / portTICK_RATE_MS);

    lcd_cmd(TFT_CMD_SWRESET);
    vTaskDelay(10 / portTICK_RATE_MS);
    lcd_cmd(TFT_CMD_SWRESET);

    //detect LCD type
    uint32_t lcd_id = lcd_get_id(ili_spi);
    int lcd_detected_type = 0;
    int lcd_type;

    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(lcd_init_cmds[cmd].cmd);
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
}


static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
static void send_lines(int ypos, int line)
{
    esp_err_t ret;
    int x;


//Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
#ifdef TDISPLAY
    const int sx = 52, sy = 41;
#else
    const int sx = 0, sy = 1;
#endif
    
    ypos += sy;
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=0;              //Start Col High
    trans[1].tx_data[1]=sx;             //Start Col Low
    trans[1].tx_data[2]=(sx+LINE_LEN-1)>>8;       //End Col High
    trans[1].tx_data[3]=(sx+LINE_LEN-1)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+PARALLEL_LINES-1)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+PARALLEL_LINES-1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].tx_buffer=dma_lines[line];        //finally send the line data
    trans[5].length=LINE_LEN*2*8*PARALLEL_LINES;          //Data length, in bits
    trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        //printf("ok %d %d\n", x, trans[x].length);
        ret=spi_device_queue_trans(ili_spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


// This is the function which will be called from Python as test.add_ints(a, b).
static void lcd_setup() {
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*LINE_LEN*2+8
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=5,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &ili_spi);
    ESP_ERROR_CHECK(ret);
    //Allocate memory for the pixel buffers
    for (int i=0; i<2; i++) {
        dma_lines[i]=heap_caps_malloc(LINE_LEN*PARALLEL_LINES*sizeof(uint16_t), MALLOC_CAP_DMA);
        memset(dma_lines[i], 0, LINE_LEN*PARALLEL_LINES*sizeof(uint16_t));
    }

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    ///Disable backlight
    gpio_set_level(PIN_NUM_BCKL, 0);
}

static uint8_t minc(int32_t c) {
    if(c > 255)
        return 255;
    return c;
}

static uint8_t nn(int32_t c)
{
    if (c<0)
        return 0;
    return c;
}


void ili_refresh(int contrast, int hue, int bl, int width, int height, char *data)
{
    static int setup, initialized;
    if(!setup) {
        lcd_setup();
        setup = 1;
    }
    
    if(bl != backlight) {
        backlight = !!bl;
        ///Enable backlight
        gpio_set_level(PIN_NUM_BCKL, backlight);
        if(!backlight) {
            lcd_cmd(TFT_CMD_SWRESET);
            initialized = 0;
        }
    }   

    if(!backlight) {
        initialized = 0;
        return;
    }

    if(!initialized) {
        lcd_init();
        initialized = 1;
    }

#if 1
    lcd_cmd(0x36);
    uint8_t d[1] = {0xC8};
    lcd_data(d, 1);
#endif
    
    // rebuild palette if needed
    static int last_contrast = -1, last_hue = -1;
    static uint16_t palette[256];
    contrast = 100;
    if(contrast != last_contrast || hue != last_hue) {
        last_contrast = contrast;
        last_hue = hue;
        int32_t c = contrast, h = hue;
        int32_t r, g, b;
        if(h < 85) {
            r = 85;
            g = nn(2*h - 85);
            b = nn(85 - 2*h);
        } else if(h < 170) {
            r = nn(85 - 2*(h-85));
            g = 85;
            b = nn(2*(h-85) - 85);
        } else {
            r = nn(2*(h-170) - 85);
            g = nn(85 - 2*(h-170));
            b = 85;
        }

        for(uint32_t i=0; i<256; i++) {
            uint32_t red = minc(i*c*r/8500);
            uint32_t green = minc(i*c*g/8500);
            uint32_t blue = minc(i*c*b/8500);


            uint16_t c = ((red&0xF8)<<8) | ((green&0xFC) << 3) | ((blue&0xF8)>>3);
            uint8_t *v = (uint8_t*)&c;
            palette[i] = (v[0]<<8) | v[1];
        }
    }
    
    //spi_device_select(ili_spi, 0);

    // put input data into dma buffer decoding palette
    int yc = 0;
    static int curline;
    for(int y=0; y<height; y++) {
        uint16_t *pl = &dma_lines[curline][yc*LINE_LEN];
        for(int x=0; x<width; x++) {
            uint8_t c = data[y*width+x];
            *pl++ = palette[c];
        }
        yc++;
        if(yc == PARALLEL_LINES || y == height-1) {
            send_lines(y-PARALLEL_LINES, curline);
            curline = !curline;
            yc = 0;
        }
    }
}
