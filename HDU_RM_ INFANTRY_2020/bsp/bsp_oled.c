/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.c
 * @brief      this file contains sd card basic operating function
 * @note
 * @Version    V1.0.0
 * @Date       Jan-28-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
#include "bsp_oled.h"
#include "oledfont.h"
#include "math.h"
#include <stdio.h>
#include <stdarg.h>
#include "jy901.h"








/**
 * OLED flash Addr:
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127
**/

const unsigned char asc2_1206[95][12]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",0*/
{0x00,0x00,0x00,0x00,0x3F,0x40,0x00,0x00,0x00,0x00,0x00,0x00},/*"!",1*/
{0x00,0x00,0x30,0x00,0x40,0x00,0x30,0x00,0x40,0x00,0x00,0x00},/*""",2*/
{0x09,0x00,0x0B,0xC0,0x3D,0x00,0x0B,0xC0,0x3D,0x00,0x09,0x00},/*"#",3*/
{0x18,0xC0,0x24,0x40,0x7F,0xE0,0x22,0x40,0x31,0x80,0x00,0x00},/*"$",4*/
{0x18,0x00,0x24,0xC0,0x1B,0x00,0x0D,0x80,0x32,0x40,0x01,0x80},/*"%",5*/
{0x03,0x80,0x1C,0x40,0x27,0x40,0x1C,0x80,0x07,0x40,0x00,0x40},/*"&",6*/
{0x10,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"'",7*/
{0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x80,0x20,0x40,0x40,0x20},/*"(",8*/
{0x00,0x00,0x40,0x20,0x20,0x40,0x1F,0x80,0x00,0x00,0x00,0x00},/*")",9*/
{0x09,0x00,0x06,0x00,0x1F,0x80,0x06,0x00,0x09,0x00,0x00,0x00},/*"*",10*/
{0x04,0x00,0x04,0x00,0x3F,0x80,0x04,0x00,0x04,0x00,0x00,0x00},/*"+",11*/
{0x00,0x10,0x00,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*",",12*/
{0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x04,0x00,0x00,0x00},/*"-",13*/
{0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*".",14*/
{0x00,0x20,0x01,0xC0,0x06,0x00,0x38,0x00,0x40,0x00,0x00,0x00},/*"/",15*/
{0x1F,0x80,0x20,0x40,0x20,0x40,0x20,0x40,0x1F,0x80,0x00,0x00},/*"0",16*/
{0x00,0x00,0x10,0x40,0x3F,0xC0,0x00,0x40,0x00,0x00,0x00,0x00},/*"1",17*/
{0x18,0xC0,0x21,0x40,0x22,0x40,0x24,0x40,0x18,0x40,0x00,0x00},/*"2",18*/
{0x10,0x80,0x20,0x40,0x24,0x40,0x24,0x40,0x1B,0x80,0x00,0x00},/*"3",19*/
{0x02,0x00,0x0D,0x00,0x11,0x00,0x3F,0xC0,0x01,0x40,0x00,0x00},/*"4",20*/
{0x3C,0x80,0x24,0x40,0x24,0x40,0x24,0x40,0x23,0x80,0x00,0x00},/*"5",21*/
{0x1F,0x80,0x24,0x40,0x24,0x40,0x34,0x40,0x03,0x80,0x00,0x00},/*"6",22*/
{0x30,0x00,0x20,0x00,0x27,0xC0,0x38,0x00,0x20,0x00,0x00,0x00},/*"7",23*/
{0x1B,0x80,0x24,0x40,0x24,0x40,0x24,0x40,0x1B,0x80,0x00,0x00},/*"8",24*/
{0x1C,0x00,0x22,0xC0,0x22,0x40,0x22,0x40,0x1F,0x80,0x00,0x00},/*"9",25*/
{0x00,0x00,0x00,0x00,0x08,0x40,0x00,0x00,0x00,0x00,0x00,0x00},/*":",26*/
{0x00,0x00,0x00,0x00,0x04,0x60,0x00,0x00,0x00,0x00,0x00,0x00},/*";",27*/
{0x00,0x00,0x04,0x00,0x0A,0x00,0x11,0x00,0x20,0x80,0x40,0x40},/*"<",28*/
{0x09,0x00,0x09,0x00,0x09,0x00,0x09,0x00,0x09,0x00,0x00,0x00},/*"=",29*/
{0x00,0x00,0x40,0x40,0x20,0x80,0x11,0x00,0x0A,0x00,0x04,0x00},/*">",30*/
{0x18,0x00,0x20,0x00,0x23,0x40,0x24,0x00,0x18,0x00,0x00,0x00},/*"?",31*/
{0x1F,0x80,0x20,0x40,0x27,0x40,0x29,0x40,0x1F,0x40,0x00,0x00},/*"@",32*/
{0x00,0x40,0x07,0xC0,0x39,0x00,0x0F,0x00,0x01,0xC0,0x00,0x40},/*"A",33*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x24,0x40,0x1B,0x80,0x00,0x00},/*"B",34*/
{0x1F,0x80,0x20,0x40,0x20,0x40,0x20,0x40,0x30,0x80,0x00,0x00},/*"C",35*/
{0x20,0x40,0x3F,0xC0,0x20,0x40,0x20,0x40,0x1F,0x80,0x00,0x00},/*"D",36*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x2E,0x40,0x30,0xC0,0x00,0x00},/*"E",37*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x2E,0x00,0x30,0x00,0x00,0x00},/*"F",38*/
{0x0F,0x00,0x10,0x80,0x20,0x40,0x22,0x40,0x33,0x80,0x02,0x00},/*"G",39*/
{0x20,0x40,0x3F,0xC0,0x04,0x00,0x04,0x00,0x3F,0xC0,0x20,0x40},/*"H",40*/
{0x20,0x40,0x20,0x40,0x3F,0xC0,0x20,0x40,0x20,0x40,0x00,0x00},/*"I",41*/
{0x00,0x60,0x20,0x20,0x20,0x20,0x3F,0xC0,0x20,0x00,0x20,0x00},/*"J",42*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x0B,0x00,0x30,0xC0,0x20,0x40},/*"K",43*/
{0x20,0x40,0x3F,0xC0,0x20,0x40,0x00,0x40,0x00,0x40,0x00,0xC0},/*"L",44*/
{0x3F,0xC0,0x3C,0x00,0x03,0xC0,0x3C,0x00,0x3F,0xC0,0x00,0x00},/*"M",45*/
{0x20,0x40,0x3F,0xC0,0x0C,0x40,0x23,0x00,0x3F,0xC0,0x20,0x00},/*"N",46*/
{0x1F,0x80,0x20,0x40,0x20,0x40,0x20,0x40,0x1F,0x80,0x00,0x00},/*"O",47*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x24,0x00,0x18,0x00,0x00,0x00},/*"P",48*/
{0x1F,0x80,0x21,0x40,0x21,0x40,0x20,0xE0,0x1F,0xA0,0x00,0x00},/*"Q",49*/
{0x20,0x40,0x3F,0xC0,0x24,0x40,0x26,0x00,0x19,0xC0,0x00,0x40},/*"R",50*/
{0x18,0xC0,0x24,0x40,0x24,0x40,0x22,0x40,0x31,0x80,0x00,0x00},/*"S",51*/
{0x30,0x00,0x20,0x40,0x3F,0xC0,0x20,0x40,0x30,0x00,0x00,0x00},/*"T",52*/
{0x20,0x00,0x3F,0x80,0x00,0x40,0x00,0x40,0x3F,0x80,0x20,0x00},/*"U",53*/
{0x20,0x00,0x3E,0x00,0x01,0xC0,0x07,0x00,0x38,0x00,0x20,0x00},/*"V",54*/
{0x38,0x00,0x07,0xC0,0x3C,0x00,0x07,0xC0,0x38,0x00,0x00,0x00},/*"W",55*/
{0x20,0x40,0x39,0xC0,0x06,0x00,0x39,0xC0,0x20,0x40,0x00,0x00},/*"X",56*/
{0x20,0x00,0x38,0x40,0x07,0xC0,0x38,0x40,0x20,0x00,0x00,0x00},/*"Y",57*/
{0x30,0x40,0x21,0xC0,0x26,0x40,0x38,0x40,0x20,0xC0,0x00,0x00},/*"Z",58*/
{0x00,0x00,0x00,0x00,0x7F,0xE0,0x40,0x20,0x40,0x20,0x00,0x00},/*"[",59*/
{0x00,0x00,0x70,0x00,0x0C,0x00,0x03,0x80,0x00,0x40,0x00,0x00},/*"\",60*/
{0x00,0x00,0x40,0x20,0x40,0x20,0x7F,0xE0,0x00,0x00,0x00,0x00},/*"]",61*/
{0x00,0x00,0x20,0x00,0x40,0x00,0x20,0x00,0x00,0x00,0x00,0x00},/*"^",62*/
{0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10},/*"_",63*/
{0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"`",64*/
{0x00,0x00,0x02,0x80,0x05,0x40,0x05,0x40,0x03,0xC0,0x00,0x40},/*"a",65*/
{0x20,0x00,0x3F,0xC0,0x04,0x40,0x04,0x40,0x03,0x80,0x00,0x00},/*"b",66*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x04,0x40,0x06,0x40,0x00,0x00},/*"c",67*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x24,0x40,0x3F,0xC0,0x00,0x40},/*"d",68*/
{0x00,0x00,0x03,0x80,0x05,0x40,0x05,0x40,0x03,0x40,0x00,0x00},/*"e",69*/
{0x00,0x00,0x04,0x40,0x1F,0xC0,0x24,0x40,0x24,0x40,0x20,0x00},/*"f",70*/
{0x00,0x00,0x02,0xE0,0x05,0x50,0x05,0x50,0x06,0x50,0x04,0x20},/*"g",71*/
{0x20,0x40,0x3F,0xC0,0x04,0x40,0x04,0x00,0x03,0xC0,0x00,0x40},/*"h",72*/
{0x00,0x00,0x04,0x40,0x27,0xC0,0x00,0x40,0x00,0x00,0x00,0x00},/*"i",73*/
{0x00,0x10,0x00,0x10,0x04,0x10,0x27,0xE0,0x00,0x00,0x00,0x00},/*"j",74*/
{0x20,0x40,0x3F,0xC0,0x01,0x40,0x07,0x00,0x04,0xC0,0x04,0x40},/*"k",75*/
{0x20,0x40,0x20,0x40,0x3F,0xC0,0x00,0x40,0x00,0x40,0x00,0x00},/*"l",76*/
{0x07,0xC0,0x04,0x00,0x07,0xC0,0x04,0x00,0x03,0xC0,0x00,0x00},/*"m",77*/
{0x04,0x40,0x07,0xC0,0x04,0x40,0x04,0x00,0x03,0xC0,0x00,0x40},/*"n",78*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x04,0x40,0x03,0x80,0x00,0x00},/*"o",79*/
{0x04,0x10,0x07,0xF0,0x04,0x50,0x04,0x40,0x03,0x80,0x00,0x00},/*"p",80*/
{0x00,0x00,0x03,0x80,0x04,0x40,0x04,0x50,0x07,0xF0,0x00,0x10},/*"q",81*/
{0x04,0x40,0x07,0xC0,0x02,0x40,0x04,0x00,0x04,0x00,0x00,0x00},/*"r",82*/
{0x00,0x00,0x06,0x40,0x05,0x40,0x05,0x40,0x04,0xC0,0x00,0x00},/*"s",83*/
{0x00,0x00,0x04,0x00,0x1F,0x80,0x04,0x40,0x00,0x40,0x00,0x00},/*"t",84*/
{0x04,0x00,0x07,0x80,0x00,0x40,0x04,0x40,0x07,0xC0,0x00,0x40},/*"u",85*/
{0x04,0x00,0x07,0x00,0x04,0xC0,0x01,0x80,0x06,0x00,0x04,0x00},/*"v",86*/
{0x06,0x00,0x01,0xC0,0x07,0x00,0x01,0xC0,0x06,0x00,0x00,0x00},/*"w",87*/
{0x04,0x40,0x06,0xC0,0x01,0x00,0x06,0xC0,0x04,0x40,0x00,0x00},/*"x",88*/
{0x04,0x10,0x07,0x10,0x04,0xE0,0x01,0x80,0x06,0x00,0x04,0x00},/*"y",89*/
{0x00,0x00,0x04,0x40,0x05,0xC0,0x06,0x40,0x04,0x40,0x00,0x00},/*"z",90*/
{0x00,0x00,0x00,0x00,0x04,0x00,0x7B,0xE0,0x40,0x20,0x00,0x00},/*"{",91*/
{0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xF0,0x00,0x00,0x00,0x00},/*"|",92*/
{0x00,0x00,0x40,0x20,0x7B,0xE0,0x04,0x00,0x00,0x00,0x00,0x00},/*"}",93*/
{0x40,0x00,0x80,0x00,0x40,0x00,0x20,0x00,0x20,0x00,0x40,0x00},/*"~",94*/
};

const unsigned char gImage_logo[832] = { /* 0X00,0X01,0X63,0X00,0X40,0X00, */
0X00,0X00,0X00,0X3F,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X3F,0XFF,0XFF,0XFF,0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFF,0XFF,
0XFF,0XFF,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,0XF8,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XFF,0XFF,0XFF,0XFC,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X07,0XFF,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,
0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,
0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFE,0X00,0X03,0XFF,0XC0,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X7F,0XFE,0X00,0X01,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X3F,0XFF,0X00,0X00,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFF,0X80,
0X00,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0XC0,0X00,0XFF,0XC0,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0XE0,0X00,0XFF,0XC0,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X03,0XFF,0XF0,0X01,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X03,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,
0XFF,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,
0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0X80,0X00,0X00,
0X00,0X00,0X00,0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X01,0XFF,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,
0XFF,0XFF,0XFF,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,
0XF8,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XF0,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X03,0XFF,0XFF,0XFF,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X07,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,
0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0X80,0X3F,
0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X07,0XFF,0X80,0X1F,0XFF,0X80,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0XFF,0X80,0X0F,0XFF,0XC0,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X0F,0XFF,0X80,0X07,0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X0F,0XFF,0X00,0X07,0XFF,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFF,0X00,
0X03,0XFF,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFF,0X00,0X01,0XFF,0XF8,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XFF,0X00,0X00,0XFF,0XFC,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X1F,0XFE,0X00,0X00,0X7F,0XFE,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X3F,0XFE,0X01,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0XFE,
0X00,0X7F,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0XFC,0X00,0X1F,0XFF,
0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0XFC,0X00,0X07,0XFF,0XFF,0XE0,0X00,
0X00,0X00,0X00,0X00,0X00,0X7F,0XFC,0X00,0X01,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XFF,0XFC,0X00,0X00,0X7F,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X1F,0XE0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X07,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XF8,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X7C,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1C,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X06,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X7F,0X07,0X87,0XF0,0XF0,0XE1,0X80,0XC0,0X7F,0X7F,0XBF,
0XBF,0XC0,0X7F,0XDF,0XEF,0XFB,0XFC,0XE1,0X81,0XE0,0XFE,0XFF,0X7F,0X3F,0XE0,0X00,
0XD8,0X60,0X1B,0X0C,0XE3,0X81,0XE1,0X80,0X18,0X00,0X00,0X60,0X01,0XF8,0X6F,0XF6,
0X0D,0XB6,0X83,0X30,0XFF,0X18,0X7F,0X00,0XE0,0XFF,0X38,0X6C,0X37,0X0D,0XBC,0XC6,
0X30,0X03,0X38,0X00,0X3F,0XC0,0XC3,0X1F,0XCF,0XF3,0XFB,0X3C,0XCD,0XF8,0XFF,0X30,
0X7F,0X61,0X80,0XC3,0X8F,0X8F,0XE1,0XF3,0X18,0XDB,0XF9,0XFE,0X30,0XFE,0X60,0XC0,
};

const unsigned char LOGO_BMP[128][8] = {
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x72},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7C},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x40,0x00,0x00,0x00,0x00,0x00,0x00,0xFE},
{0x60,0x00,0x00,0x00,0x00,0x10,0x00,0x6C},
{0x70,0x00,0x00,0x00,0x00,0x30,0x00,0x00},
{0x78,0x00,0x00,0x00,0x00,0xF0,0x00,0x7C},
{0x7C,0x00,0x00,0x00,0x07,0xF0,0x00,0xFE},
{0x7E,0x00,0x00,0x00,0x3F,0xF0,0x00,0xC6},
{0x7F,0x00,0x00,0x01,0xFF,0xF0,0x00,0xC6},
{0x7F,0x80,0x00,0x0F,0xFF,0xF0,0x00,0xC6},
{0x7F,0xC0,0x00,0x7F,0xFF,0xF0,0x00,0xFE},
{0x7F,0xE0,0x03,0xFF,0xFF,0xF0,0x00,0x7C},
{0x7F,0xF0,0x3F,0xFF,0xFF,0xF0,0x00,0x02},
{0x7F,0xF8,0x3F,0xFF,0xFF,0xF0,0x00,0x06},
{0x7F,0xFC,0x3F,0xFF,0xFF,0xF0,0x00,0x1E},
{0x7F,0xFE,0x3F,0xFF,0xFF,0xF0,0x00,0xBC},
{0x7F,0xFF,0x3F,0xFF,0xFF,0xF0,0x00,0xE0},
{0x7F,0xFF,0xBF,0xFF,0xFF,0x80,0x00,0xF8},
{0x7F,0xFF,0xFF,0xFF,0xFC,0x00,0x00,0x3E},
{0x7F,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x0E},
{0x7F,0xFF,0xFF,0xFF,0x00,0x00,0x00,0xB8},
{0x7F,0xFF,0xFF,0xF8,0x00,0x00,0x00,0xE0},
{0x7F,0xFF,0xFF,0xF0,0x00,0x00,0x00,0xFE},
{0x7F,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x1E},
{0x7F,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x02},
{0x7F,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00},
{0x7F,0xEF,0xFF,0xF0,0x02,0x00,0x00,0x06},
{0x7F,0xE7,0xFF,0xF0,0x02,0x00,0x00,0x0E},
{0x7F,0xE3,0xFF,0xF0,0x02,0x00,0x00,0x1C},
{0x7F,0xE1,0xFF,0xF8,0x03,0x00,0x00,0xBA},
{0x7F,0xE0,0xFF,0xFC,0x03,0x00,0x00,0xF6},
{0x7F,0xE0,0x7F,0xFE,0x03,0x80,0x00,0xE6},
{0x7F,0xE0,0x3F,0xFF,0x03,0x80,0x00,0xF6},
{0x7F,0xE0,0x3F,0xFF,0x83,0xC0,0x00,0x3E},
{0x7F,0xE0,0x3F,0xFF,0xC3,0xC0,0x00,0x0E},
{0x7F,0xE0,0x3F,0xFF,0xE3,0xE0,0x00,0x02},
{0x7F,0xE0,0x3F,0xFF,0xF3,0xE0,0x00,0x00},
{0x7F,0xE0,0x3F,0xFF,0xFB,0xF0,0x00,0x02},
{0x7F,0xE0,0x3F,0xFF,0xFF,0xF0,0x00,0x66},
{0x7F,0xE0,0x3F,0xFF,0xFF,0xF8,0x00,0xF6},
{0x7F,0xE0,0x3F,0xFF,0xFF,0xF8,0x00,0xD6},
{0x7F,0xE0,0x3F,0xFF,0xFF,0xFC,0x00,0xD6},
{0x7F,0xF0,0x7F,0xFF,0xFF,0xFC,0x00,0xD6},
{0x7F,0xF8,0xFF,0xF7,0xFF,0xFE,0x00,0xD6},
{0x7F,0xFF,0xFF,0xF3,0xFF,0xFE,0x00,0xDE},
{0x3F,0xFF,0xFF,0xE1,0xFF,0xFF,0x00,0x8C},
{0x3F,0xFF,0xFF,0xE0,0xFF,0xCF,0x00,0x40},
{0x1F,0xFF,0xFF,0xC0,0x7F,0xC7,0x80,0xC0},
{0x1F,0xFF,0xFF,0xC0,0x3F,0xC3,0x80,0xC0},
{0x0F,0xFF,0xFF,0x80,0x1F,0xC1,0xC0,0xFE},
{0x07,0xFF,0xFF,0x00,0x0F,0xC0,0xC0,0xFE},
{0x03,0xFF,0xFE,0x00,0x07,0xC0,0x60,0xC0},
{0x01,0xFF,0xFC,0x00,0x03,0xC0,0x20,0xC0},
{0x00,0x7F,0xF0,0x00,0x01,0xC0,0x00,0x86},
{0x00,0x0F,0x80,0x00,0x00,0xC0,0x00,0x16},
{0x00,0x00,0x00,0x00,0x00,0x40,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD0},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC0},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x72},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x62},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD6},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDE},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8C},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};
static uint8_t OLED_GRAM[131][8];
//short delay uesd in spi transmmit
void delay_ms(uint16_t delaytimes)
{
    uint16_t i;
    for (i = 0; i < delaytimes; i++ )
    {
        int a = 10000;  //delay based on mian clock, 168Mhz
        while (a-- );
    }
}


/**
 * @brief   write data/command to OLED
 * @param   dat: the data ready to write
 * @param   cmd: 0x00,command 0x01,data
 * @retval
 */
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    if (cmd != 0)
        OLED_CMD_Set();
    else
        OLED_CMD_Clr();

    HAL_SPI_Transmit(&hspi1, &dat, 1, 10);
}


/**
 * @brief   set OLED cursor position
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @retval
 */
static void oled_set_pos(uint8_t x, uint8_t y)
{
    x += 2;
    oled_write_byte((0xb0 + y), OLED_CMD);              //set page address y
    oled_write_byte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address
    oled_write_byte((x&0xf0), OLED_CMD);                //set column low address
}

/**
 * @brief   turn on OLED display
 * @param   None
 * @param   None
 * @retval
 */
void oled_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief   turn off OLED display
 * @param   None
 * @param   None
 * @retval
 */
void oled_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
 * @brief   refresh the RAM of OLED
 * @param   None
 * @param   None
 * @retval
 */
void oled_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        oled_set_pos(0, i);

        for (n = 0; n < 131; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}

/**
 * @brief   clear the screen
 * @param   None
 * @param   None
 * @retval
 */
void oled_clear(Pen_Typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == Pen_Write)
                OLED_GRAM[n][i] = 0xff;
            else if (pen == Pen_Clear)
                OLED_GRAM[n][i] = 0x00;
            else
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
        }
    }
	oled_refresh_gram();
}

/**
 * @brief   draw a point at (x, y)
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen)
{
    uint8_t page = 0, row = 0;
    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
        return;

    page = y / 8;
    row = y % 8;

    if (pen == Pen_Write)
        OLED_GRAM[x][page] |= 1 << row;
    else if (pen == Pen_Inversion)
        OLED_GRAM[x][page] ^= 1 << row;
    else
        OLED_GRAM[x][page] &= ~(1 << row);

}

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_drawpoint(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            oled_drawpoint(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_drawpoint(col, (uint8_t)(col * k + b), pen);
        }
    }
}


//To add: rectangle, fillrectangle, circle, fillcircle,

/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                oled_drawpoint(x, y, Pen_Write);
            else
                oled_drawpoint(x, y, Pen_Clear);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

//m^n
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
        result *= m;

    return result;
}

/**
 * @brief   show a number
 * @param   row: row of number
 * @param   col: column of number
 * @param   num: the number ready to show
 * @param   mode: 0x01, fill number with '0'; 0x00, fill number with spaces
 * @param   len: the length of the number
 * @retval  None
 */
void oled_shownum(uint8_t row, uint8_t col, int32_t num, uint8_t mode, uint8_t len)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    if(num>=0)
    {
		for (t = 0; t < len; t++)
		{
			temp = (num / oled_pow(10, len - t -1)) % 10;

			if (enshow == 0 && t < (len - 1))
			{
				if (temp == 0)
				{
					if (mode == 0)
						oled_showchar(row, col + t, ' ');
					else
						oled_showchar(row, col + t, '0');
					continue;
				}
				else
					enshow = 1;
			}

			oled_showchar(row, col + t, temp + '0');
		}
    }
    else
    {
    	oled_showchar(row, col , '-');
    	for (t = 0; t < len; t++)
		{
			temp = (abs(num) / oled_pow(10, len - t -1)) % 10;

			if (enshow == 0 && t < (len - 1))
			{
				if (temp == 0)
				{
					if (mode == 0)
						oled_showchar(row, col + t +1, ' ');
					else
						oled_showchar(row, col + t +1, '0');
					continue;
				}
				else
					enshow = 1;
			}

			oled_showchar(row, col + t, temp + '0');
		}
    }
    oled_refresh_gram();
}


/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        oled_showchar(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
    oled_refresh_gram();
}

/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void oled_printf(uint8_t row, uint8_t col, const char *fmt,...)
{
    uint8_t LCD_BUF[128] = {0};
    uint8_t remain_size = 0;
    va_list ap;

    if ((row > 4) || (row < 1) || (col > 20) || (col < 1))
        return;

    va_start(ap, fmt);

    vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    oled_showstring(row, col, LCD_BUF);

}



/**
 * @brief   initialize the oled module
 * @param   None
 * @retval  None
 */
void oled_init(void)
{
    OLED_RST_Clr();
    HAL_Delay(500);
    OLED_RST_Set();

    oled_write_byte(0xae, OLED_CMD);    //turn off oled panel
    oled_write_byte(0x00, OLED_CMD);    //set low column address
    oled_write_byte(0x10, OLED_CMD);    //set high column address
    oled_write_byte(0x40, OLED_CMD);    //set start line address
    oled_write_byte(0x81, OLED_CMD);    //set contrast control resigter
    oled_write_byte(0xcf, OLED_CMD);    //set SEG output current brightness
    oled_write_byte(0xa1, OLED_CMD);    //set SEG/column mapping
    oled_write_byte(0xc8, OLED_CMD);    //set COM/row scan direction
    oled_write_byte(0xa6, OLED_CMD);    //set nomarl display
    oled_write_byte(0xa8, OLED_CMD);    //set multiplex display
    oled_write_byte(0x3f, OLED_CMD);    //1/64 duty
    oled_write_byte(0xd3, OLED_CMD);    //set display offset
    oled_write_byte(0x00, OLED_CMD);    //not offest
    oled_write_byte(0xd5, OLED_CMD);    //set display clock divide ratio/oscillator frequency
    oled_write_byte(0x80, OLED_CMD);    //set divide ratio
    oled_write_byte(0xd9, OLED_CMD);    //set pre-charge period
    oled_write_byte(0xf1, OLED_CMD);    //pre-charge: 15 clocks, discharge: 1 clock
    oled_write_byte(0xda, OLED_CMD);    //set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);    //
    oled_write_byte(0xdb, OLED_CMD);    //set vcomh
    oled_write_byte(0x40, OLED_CMD);    //set vcom deselect level
    oled_write_byte(0x20, OLED_CMD);    //set page addressing mode
    oled_write_byte(0x02, OLED_CMD);    //
    oled_write_byte(0x8d, OLED_CMD);    //set charge pump enable/disable
    oled_write_byte(0x14, OLED_CMD);    //charge pump disable
    oled_write_byte(0xa4, OLED_CMD);    //disable entire dispaly on
    oled_write_byte(0xa6, OLED_CMD);    //disable inverse display on
    oled_write_byte(0xaf, OLED_CMD);    //turn on oled panel

    oled_write_byte(0xaf, OLED_CMD);    //display on

    oled_clear(Pen_Clear);
    oled_set_pos(0, 0);

}

uint8_t OledKeyValue=0x00;
void getOledKeyValue(void)
{
	if(ADCxConvertedValue<=100)
	{
		OledKeyValue=key_middle;
	}
	if(ADCxConvertedValue>=850&&ADCxConvertedValue<=950)
	{
		OledKeyValue=key_left;
	}
	if(ADCxConvertedValue>=1600&&ADCxConvertedValue<=1700)
	{
		OledKeyValue=key_right;
	}
	if(ADCxConvertedValue>=2000&&ADCxConvertedValue<=2300)
	{
		OledKeyValue=key_up;
	}
	if(ADCxConvertedValue>=2600&&ADCxConvertedValue<=2900)
	{
		OledKeyValue=key_down;
	}
	if(ADCxConvertedValue>=3000)
	{
		OledKeyValue=key_no;
	}
}

u8 KEY_Scan(void)
{
	getOledKeyValue();
	static u8 key_Up=1;//按键按松开标志

	if(key_Up&&(OledKeyValue!=key_no))
	{
		key_Up=0;
		if(OledKeyValue==key_up)return key_up_pres;
		else if(OledKeyValue==key_down)return key_down_pres;
		else if(OledKeyValue==key_left)return key_left_pres;
		else if(OledKeyValue==key_right)return key_right_pres;
		else if(OledKeyValue==key_middle)return key_middle_pres;
	}else if(OledKeyValue==key_no)key_Up=1;
 	return 0;// 无按键按下
}
#include <stdio.h>
#include <stdint.h>

static char table[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

void num2char(char *str, double number, uint8_t g, uint8_t l)
{
	if(number>0)
	{
		uint8_t i;
		int temp = number/1;
		double t2 = 0.0;
		for (i = 1; i<=g; i++)
		{
			if (temp==0)
				str[g-i] = table[0];
			else
				str[g-i] = table[temp%10];
			temp = temp/10;
		}
		*(str+g) = '.';
		temp = 0;
		t2 = number;
		for(i=1; i<=l; i++)
		{
			temp = t2*10;
			str[g+i] = table[temp%10];
			t2 = t2*10;
		}
		*(str+g+l+1) = '\0';
	}
	else
	{
		uint8_t i;
		int temp = fabs(number)/1;
		double t2 = 0.0;
		str[0]='-';
		for (i = 1; i<=g; i++)
		{
			if (temp==0)
				str[1+g-i] = table[0];
			else
				str[1+g-i] = table[temp%10];
			temp = temp/10;
		}
		*(str+g+1) = '.';
		temp = 0;
		t2 = fabs(number);
		for(i=1; i<=l; i++)
		{
			temp = t2*10;
			str[1+g+i] = table[temp%10];
			t2 = t2*10;
		}
		*(str+g+l+2) = '\0';
	}
}
void oled_showfloat(uint8_t row, uint8_t col,double value ,uint8_t g, uint8_t l)
{
	uint8_t temp[30];
	num2char(temp,value,g,l);
	oled_showstring(row,col,temp);
}