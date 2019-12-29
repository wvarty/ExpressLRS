#pragma once


#define EMPTY() 


#ifndef ICACHE_RAM_ATTR //fix to allow both esp32 and esp8266
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif

#ifdef PLATFORM_STM32
#define ICACHE_RAM_ATTR //nothing//
#endif


#ifdef TARGET_TTGO_LORA_V1_AS_TX
#define GPIO_PIN_NSS 18
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 5
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE -1
#define GPIO_PIN_TX_ENABLE -1
#define GPIO_PIN_OLED_SDA 4
#define GPIO_PIN_OLED_SCK 15
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TTGO_LORA_V1_AS_RX
#endif

#ifdef TARGET_TTGO_LORA_V2_AS_TX
#define GPIO_PIN_NSS 18
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 5
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE -1
#define GPIO_PIN_TX_ENABLE -1
#define GPIO_PIN_OLED_SDA 21
#define GPIO_PIN_OLED_SCK 22
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TTGO_LORA_V2_AS_RX
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3
#define GPIO_PIN_NSS 5
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 25
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE 13
#define GPIO_PIN_TX_ENABLE 12
#define GPIO_PIN_OLED_SDA -1
#define GPIO_PIN_OLED_SCK -1
#define GPIO_PIN_RCSIGNAL_RX 2
#define GPIO_PIN_RCSIGNAL_TX 4
#endif

#ifdef TARGET_EXPRESSLRS_PCB_RX_V3
#define GPIO_PIN_NSS 15
#define GPIO_PIN_DIO0 4
#define GPIO_PIN_DIO1 5
#define GPIO_PIN_MOSI 13
#define GPIO_PIN_MISO 12
#define GPIO_PIN_SCK 14
#define GPIO_PIN_RST 2
#define GPIO_PIN_RX_ENABLE -1
#define GPIO_PIN_TX_ENABLE -1
#define GPIO_PIN_OLED_SDA -1
#define GPIO_PIN_OLED_SCK -1
#define GPIO_PIN_RCSIGNAL_RX -1 //not relevant, can use only default for esp8266=esp8285
#define GPIO_PIN_RCSIGNAL_TX -1
#endif

#ifdef TARGET_R9M_RX
#define GPIO_PIN_NSS -1 // not sure if for thesis so no ascii dwg this time - Chris
#define GPIO_PIN_DIO0 -1
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI PB15
#define GPIO_PIN_MISO -1
#define GPIO_PIN_SCK -1
#define GPIO_PIN_RST -1
#define GPIO_PIN_RX_ENABLE -1
#define GPIO_PIN_TX_ENABLE -1
#define GPIO_PIN_OLED_SDA -1
#define GPIO_PIN_OLED_SCK -1
#define GPIO_PIN_RCSIGNAL_RX PA10 //not relevant, can use only default for esp8266=esp8285
#define GPIO_PIN_RCSIGNAL_TX PA9
#define GPIO_PIN_LED C1
#endif

// #sbus 1/3 A2
// #sport/fport 1/6 AND 2/11 A5 and B10
// #inverted sport 2/12 B11
// #ch4 1/11 A10
// #ch3 1/10 A9
// #ch2 1/12 A11
// #ch1 1/9 A8
// #red led C1
// #nrst 3/15 C14
// #chip select 2/13 B12
// #MOSI 2/16 B15
// #MISO 2/13 AND 2/15 B12 and B14
// #CK 2/14 B13