/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Motor control tables.
 *
 *      This file contains the table definitions used for motor control.
 *
 * \par Application note:
 *      AVR447: Sinusoidal driving of three-phase permanent motor using
 *      ATmega48/88/168
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.4 $
 * $RCSfile: PMSMtables.h,v $
 * $Date: 2006/03/16 11:57:20 $  \n
 ******************************************************************************/

#ifndef _PMSM_TABLES_H_
#define _PMSM_TABLES_H_

/*! \brief Sine wave modulation table
 *
 *  Table containing modulation values for all three phases.
 *  The table is organized as [U1, V1, W1, U2, V2, W2, ...]
 *  for forward driving, and [U1, W1, V1, U2, W2, V2, ...]
 *  for reverse driving.
 */
__flash const uint8_t sineTable[SINE_TABLE_LENGTH * 3] = {
0,       0,      221,
8,       0,      225,
17,      0,      229,
25,      0,      232,
33,      0,      236,
42,      0,      239,
50,      0,      241,
58,      0,      244,
66,      0,      246,
74,      0,      248,
82,      0,      250,
90,      0,      252,
98,      0,      253,
105,     0,      254,
113,     0,      254,
120,     0,      255,
128,     0,      255,
135,     0,      255,
142,     0,      254,
149,     0,      254,
155,     0,      253,
162,     0,      252,
168,     0,      250,
174,     0,      248,
180,     0,      246,
186,     0,      244,
192,     0,      241,
197,     0,      239,
202,     0,      236,
207,     0,      232,
212,     0,      229,
217,     0,      225,
221,     0,      221,
225,     0,      217,
229,     0,      212,
232,     0,      207,
236,     0,      202,
239,     0,      197,
241,     0,      192,
244,     0,      186,
246,     0,      180,
248,     0,      174,
250,     0,      168,
252,     0,      162,
253,     0,      155,
254,     0,      149,
254,     0,      142,
255,     0,      135,
255,     0,      127,
255,     0,      120,
254,     0,      113,
254,     0,      105,
253,     0,      98,
252,     0,      90,
250,     0,      82,
248,     0,      74,
246,     0,      66,
244,     0,      58,
241,     0,      50,
239,     0,      42,
236,     0,      33,
232,     0,      25,
229,     0,      17,
225,     0,      8,
221,     0,      0,
225,     8,      0,
229,     17,     0,
232,     25,     0,
236,     33,     0,
239,     42,     0,
241,     50,     0,
244,     58,     0,
246,     66,     0,
248,     74,     0,
250,     82,     0,
252,     90,     0,
253,     98,     0,
254,     105,    0,
254,     113,    0,
255,     120,    0,
255,     127,    0,
255,     135,    0,
254,     142,    0,
254,     149,    0,
253,     155,    0,
252,     162,    0,
250,     168,    0,
248,     174,    0,
246,     180,    0,
244,     186,    0,
241,     192,    0,
239,     197,    0,
236,     202,    0,
232,     207,    0,
229,     212,    0,
225,     217,    0,
221,     221,    0,
217,     225,    0,
212,     229,    0,
207,     232,    0,
202,     236,    0,
197,     239,    0,
192,     241,    0,
186,     244,    0,
180,     246,    0,
174,     248,    0,
168,     250,    0,
162,     252,    0,
155,     253,    0,
149,     254,    0,
142,     254,    0,
135,     255,    0,
128,     255,    0,
120,     255,    0,
113,     254,    0,
105,     254,    0,
98,      253,    0,
90,      252,    0,
82,      250,    0,
74,      248,    0,
66,      246,    0,
58,      244,    0,
50,      241,    0,
42,      239,    0,
33,      236,    0,
25,      232,    0,
17,      229,    0,
8,       225,    0,
0,       221,    0,
0,       225,    8,
0,       229,    17,
0,       232,    25,
0,       236,    33,
0,       239,    42,
0,       241,    50,
0,       244,    58,
0,       246,    66,
0,       248,    74,
0,       250,    82,
0,       252,    90,
0,       253,    98,
0,       254,    105,
0,       254,    113,
0,       255,    120,
0,       255,    128,
0,       255,    135,
0,       254,    142,
0,       254,    149,
0,       253,    155,
0,       252,    162,
0,       250,    168,
0,       248,    174,
0,       246,    180,
0,       244,    186,
0,       241,    192,
0,       239,    197,
0,       236,    202,
0,       232,    207,
0,       229,    212,
0,       225,    217,
0,       221,    221,
0,       217,    225,
0,       212,    229,
0,       207,    232,
0,       202,    236,
0,       197,    239,
0,       192,    241,
0,       186,    244,
0,       180,    246,
0,       174,    248,
0,       168,    250,
0,       162,    252,
0,       155,    253,
0,       149,    254,
0,       142,    254,
0,       135,    255,
0,       128,    255,
0,       120,    255,
0,       113,    254,
0,       105,    254,
0,       98,     253,
0,       90,     252,
0,       82,     250,
0,       74,     248,
0,       66,     246,
0,       58,     244,
0,       50,     241,
0,       42,     239,
0,       33,     236,
0,       25,     232,
0,       17,     229,
0,       8,      225
};


/*! \brief Divistion table used to obtain sine table increment without
 *  performing a division at high motor speed.
 *
 *  This table contains 8.8 fixed point step size values
 *  for 'ticks' less than 256.
 */
__flash const uint16_t divisionTable[256] = {
0,
((SINE_TABLE_LENGTH / 6) << 8) / 1,
((SINE_TABLE_LENGTH / 6) << 8) / 2,
((SINE_TABLE_LENGTH / 6) << 8) / 3,
((SINE_TABLE_LENGTH / 6) << 8) / 4,
((SINE_TABLE_LENGTH / 6) << 8) / 5,
((SINE_TABLE_LENGTH / 6) << 8) / 6,
((SINE_TABLE_LENGTH / 6) << 8) / 7,
((SINE_TABLE_LENGTH / 6) << 8) / 8,
((SINE_TABLE_LENGTH / 6) << 8) / 9,
((SINE_TABLE_LENGTH / 6) << 8) / 10,
((SINE_TABLE_LENGTH / 6) << 8) / 11,
((SINE_TABLE_LENGTH / 6) << 8) / 12,
((SINE_TABLE_LENGTH / 6) << 8) / 13,
((SINE_TABLE_LENGTH / 6) << 8) / 14,
((SINE_TABLE_LENGTH / 6) << 8) / 15,
((SINE_TABLE_LENGTH / 6) << 8) / 16,
((SINE_TABLE_LENGTH / 6) << 8) / 17,
((SINE_TABLE_LENGTH / 6) << 8) / 18,
((SINE_TABLE_LENGTH / 6) << 8) / 19,
((SINE_TABLE_LENGTH / 6) << 8) / 20,
((SINE_TABLE_LENGTH / 6) << 8) / 21,
((SINE_TABLE_LENGTH / 6) << 8) / 22,
((SINE_TABLE_LENGTH / 6) << 8) / 23,
((SINE_TABLE_LENGTH / 6) << 8) / 24,
((SINE_TABLE_LENGTH / 6) << 8) / 25,
((SINE_TABLE_LENGTH / 6) << 8) / 26,
((SINE_TABLE_LENGTH / 6) << 8) / 27,
((SINE_TABLE_LENGTH / 6) << 8) / 28,
((SINE_TABLE_LENGTH / 6) << 8) / 29,
((SINE_TABLE_LENGTH / 6) << 8) / 30,
((SINE_TABLE_LENGTH / 6) << 8) / 31,
((SINE_TABLE_LENGTH / 6) << 8) / 32,
((SINE_TABLE_LENGTH / 6) << 8) / 33,
((SINE_TABLE_LENGTH / 6) << 8) / 34,
((SINE_TABLE_LENGTH / 6) << 8) / 35,
((SINE_TABLE_LENGTH / 6) << 8) / 36,
((SINE_TABLE_LENGTH / 6) << 8) / 37,
((SINE_TABLE_LENGTH / 6) << 8) / 38,
((SINE_TABLE_LENGTH / 6) << 8) / 39,
((SINE_TABLE_LENGTH / 6) << 8) / 40,
((SINE_TABLE_LENGTH / 6) << 8) / 41,
((SINE_TABLE_LENGTH / 6) << 8) / 42,
((SINE_TABLE_LENGTH / 6) << 8) / 43,
((SINE_TABLE_LENGTH / 6) << 8) / 44,
((SINE_TABLE_LENGTH / 6) << 8) / 45,
((SINE_TABLE_LENGTH / 6) << 8) / 46,
((SINE_TABLE_LENGTH / 6) << 8) / 47,
((SINE_TABLE_LENGTH / 6) << 8) / 48,
((SINE_TABLE_LENGTH / 6) << 8) / 49,
((SINE_TABLE_LENGTH / 6) << 8) / 50,
((SINE_TABLE_LENGTH / 6) << 8) / 51,
((SINE_TABLE_LENGTH / 6) << 8) / 52,
((SINE_TABLE_LENGTH / 6) << 8) / 53,
((SINE_TABLE_LENGTH / 6) << 8) / 54,
((SINE_TABLE_LENGTH / 6) << 8) / 55,
((SINE_TABLE_LENGTH / 6) << 8) / 56,
((SINE_TABLE_LENGTH / 6) << 8) / 57,
((SINE_TABLE_LENGTH / 6) << 8) / 58,
((SINE_TABLE_LENGTH / 6) << 8) / 59,
((SINE_TABLE_LENGTH / 6) << 8) / 60,
((SINE_TABLE_LENGTH / 6) << 8) / 61,
((SINE_TABLE_LENGTH / 6) << 8) / 62,
((SINE_TABLE_LENGTH / 6) << 8) / 63,
((SINE_TABLE_LENGTH / 6) << 8) / 64,
((SINE_TABLE_LENGTH / 6) << 8) / 65,
((SINE_TABLE_LENGTH / 6) << 8) / 66,
((SINE_TABLE_LENGTH / 6) << 8) / 67,
((SINE_TABLE_LENGTH / 6) << 8) / 68,
((SINE_TABLE_LENGTH / 6) << 8) / 69,
((SINE_TABLE_LENGTH / 6) << 8) / 70,
((SINE_TABLE_LENGTH / 6) << 8) / 71,
((SINE_TABLE_LENGTH / 6) << 8) / 72,
((SINE_TABLE_LENGTH / 6) << 8) / 73,
((SINE_TABLE_LENGTH / 6) << 8) / 74,
((SINE_TABLE_LENGTH / 6) << 8) / 75,
((SINE_TABLE_LENGTH / 6) << 8) / 76,
((SINE_TABLE_LENGTH / 6) << 8) / 77,
((SINE_TABLE_LENGTH / 6) << 8) / 78,
((SINE_TABLE_LENGTH / 6) << 8) / 79,
((SINE_TABLE_LENGTH / 6) << 8) / 80,
((SINE_TABLE_LENGTH / 6) << 8) / 81,
((SINE_TABLE_LENGTH / 6) << 8) / 82,
((SINE_TABLE_LENGTH / 6) << 8) / 83,
((SINE_TABLE_LENGTH / 6) << 8) / 84,
((SINE_TABLE_LENGTH / 6) << 8) / 85,
((SINE_TABLE_LENGTH / 6) << 8) / 86,
((SINE_TABLE_LENGTH / 6) << 8) / 87,
((SINE_TABLE_LENGTH / 6) << 8) / 88,
((SINE_TABLE_LENGTH / 6) << 8) / 89,
((SINE_TABLE_LENGTH / 6) << 8) / 90,
((SINE_TABLE_LENGTH / 6) << 8) / 91,
((SINE_TABLE_LENGTH / 6) << 8) / 92,
((SINE_TABLE_LENGTH / 6) << 8) / 93,
((SINE_TABLE_LENGTH / 6) << 8) / 94,
((SINE_TABLE_LENGTH / 6) << 8) / 95,
((SINE_TABLE_LENGTH / 6) << 8) / 96,
((SINE_TABLE_LENGTH / 6) << 8) / 97,
((SINE_TABLE_LENGTH / 6) << 8) / 98,
((SINE_TABLE_LENGTH / 6) << 8) / 99,
((SINE_TABLE_LENGTH / 6) << 8) / 100,
((SINE_TABLE_LENGTH / 6) << 8) / 101,
((SINE_TABLE_LENGTH / 6) << 8) / 102,
((SINE_TABLE_LENGTH / 6) << 8) / 103,
((SINE_TABLE_LENGTH / 6) << 8) / 104,
((SINE_TABLE_LENGTH / 6) << 8) / 105,
((SINE_TABLE_LENGTH / 6) << 8) / 106,
((SINE_TABLE_LENGTH / 6) << 8) / 107,
((SINE_TABLE_LENGTH / 6) << 8) / 108,
((SINE_TABLE_LENGTH / 6) << 8) / 109,
((SINE_TABLE_LENGTH / 6) << 8) / 110,
((SINE_TABLE_LENGTH / 6) << 8) / 111,
((SINE_TABLE_LENGTH / 6) << 8) / 112,
((SINE_TABLE_LENGTH / 6) << 8) / 113,
((SINE_TABLE_LENGTH / 6) << 8) / 114,
((SINE_TABLE_LENGTH / 6) << 8) / 115,
((SINE_TABLE_LENGTH / 6) << 8) / 116,
((SINE_TABLE_LENGTH / 6) << 8) / 117,
((SINE_TABLE_LENGTH / 6) << 8) / 118,
((SINE_TABLE_LENGTH / 6) << 8) / 119,
((SINE_TABLE_LENGTH / 6) << 8) / 120,
((SINE_TABLE_LENGTH / 6) << 8) / 121,
((SINE_TABLE_LENGTH / 6) << 8) / 122,
((SINE_TABLE_LENGTH / 6) << 8) / 123,
((SINE_TABLE_LENGTH / 6) << 8) / 124,
((SINE_TABLE_LENGTH / 6) << 8) / 125,
((SINE_TABLE_LENGTH / 6) << 8) / 126,
((SINE_TABLE_LENGTH / 6) << 8) / 127,
((SINE_TABLE_LENGTH / 6) << 8) / 128,
((SINE_TABLE_LENGTH / 6) << 8) / 129,
((SINE_TABLE_LENGTH / 6) << 8) / 130,
((SINE_TABLE_LENGTH / 6) << 8) / 131,
((SINE_TABLE_LENGTH / 6) << 8) / 132,
((SINE_TABLE_LENGTH / 6) << 8) / 133,
((SINE_TABLE_LENGTH / 6) << 8) / 134,
((SINE_TABLE_LENGTH / 6) << 8) / 135,
((SINE_TABLE_LENGTH / 6) << 8) / 136,
((SINE_TABLE_LENGTH / 6) << 8) / 137,
((SINE_TABLE_LENGTH / 6) << 8) / 138,
((SINE_TABLE_LENGTH / 6) << 8) / 139,
((SINE_TABLE_LENGTH / 6) << 8) / 140,
((SINE_TABLE_LENGTH / 6) << 8) / 141,
((SINE_TABLE_LENGTH / 6) << 8) / 142,
((SINE_TABLE_LENGTH / 6) << 8) / 143,
((SINE_TABLE_LENGTH / 6) << 8) / 144,
((SINE_TABLE_LENGTH / 6) << 8) / 145,
((SINE_TABLE_LENGTH / 6) << 8) / 146,
((SINE_TABLE_LENGTH / 6) << 8) / 147,
((SINE_TABLE_LENGTH / 6) << 8) / 148,
((SINE_TABLE_LENGTH / 6) << 8) / 149,
((SINE_TABLE_LENGTH / 6) << 8) / 150,
((SINE_TABLE_LENGTH / 6) << 8) / 151,
((SINE_TABLE_LENGTH / 6) << 8) / 152,
((SINE_TABLE_LENGTH / 6) << 8) / 153,
((SINE_TABLE_LENGTH / 6) << 8) / 154,
((SINE_TABLE_LENGTH / 6) << 8) / 155,
((SINE_TABLE_LENGTH / 6) << 8) / 156,
((SINE_TABLE_LENGTH / 6) << 8) / 157,
((SINE_TABLE_LENGTH / 6) << 8) / 158,
((SINE_TABLE_LENGTH / 6) << 8) / 159,
((SINE_TABLE_LENGTH / 6) << 8) / 160,
((SINE_TABLE_LENGTH / 6) << 8) / 161,
((SINE_TABLE_LENGTH / 6) << 8) / 162,
((SINE_TABLE_LENGTH / 6) << 8) / 163,
((SINE_TABLE_LENGTH / 6) << 8) / 164,
((SINE_TABLE_LENGTH / 6) << 8) / 165,
((SINE_TABLE_LENGTH / 6) << 8) / 166,
((SINE_TABLE_LENGTH / 6) << 8) / 167,
((SINE_TABLE_LENGTH / 6) << 8) / 168,
((SINE_TABLE_LENGTH / 6) << 8) / 169,
((SINE_TABLE_LENGTH / 6) << 8) / 170,
((SINE_TABLE_LENGTH / 6) << 8) / 171,
((SINE_TABLE_LENGTH / 6) << 8) / 172,
((SINE_TABLE_LENGTH / 6) << 8) / 173,
((SINE_TABLE_LENGTH / 6) << 8) / 174,
((SINE_TABLE_LENGTH / 6) << 8) / 175,
((SINE_TABLE_LENGTH / 6) << 8) / 176,
((SINE_TABLE_LENGTH / 6) << 8) / 177,
((SINE_TABLE_LENGTH / 6) << 8) / 178,
((SINE_TABLE_LENGTH / 6) << 8) / 179,
((SINE_TABLE_LENGTH / 6) << 8) / 180,
((SINE_TABLE_LENGTH / 6) << 8) / 181,
((SINE_TABLE_LENGTH / 6) << 8) / 182,
((SINE_TABLE_LENGTH / 6) << 8) / 183,
((SINE_TABLE_LENGTH / 6) << 8) / 184,
((SINE_TABLE_LENGTH / 6) << 8) / 185,
((SINE_TABLE_LENGTH / 6) << 8) / 186,
((SINE_TABLE_LENGTH / 6) << 8) / 187,
((SINE_TABLE_LENGTH / 6) << 8) / 188,
((SINE_TABLE_LENGTH / 6) << 8) / 189,
((SINE_TABLE_LENGTH / 6) << 8) / 190,
((SINE_TABLE_LENGTH / 6) << 8) / 191,
((SINE_TABLE_LENGTH / 6) << 8) / 192,
((SINE_TABLE_LENGTH / 6) << 8) / 193,
((SINE_TABLE_LENGTH / 6) << 8) / 194,
((SINE_TABLE_LENGTH / 6) << 8) / 195,
((SINE_TABLE_LENGTH / 6) << 8) / 196,
((SINE_TABLE_LENGTH / 6) << 8) / 197,
((SINE_TABLE_LENGTH / 6) << 8) / 198,
((SINE_TABLE_LENGTH / 6) << 8) / 199,
((SINE_TABLE_LENGTH / 6) << 8) / 200,
((SINE_TABLE_LENGTH / 6) << 8) / 201,
((SINE_TABLE_LENGTH / 6) << 8) / 202,
((SINE_TABLE_LENGTH / 6) << 8) / 203,
((SINE_TABLE_LENGTH / 6) << 8) / 204,
((SINE_TABLE_LENGTH / 6) << 8) / 205,
((SINE_TABLE_LENGTH / 6) << 8) / 206,
((SINE_TABLE_LENGTH / 6) << 8) / 207,
((SINE_TABLE_LENGTH / 6) << 8) / 208,
((SINE_TABLE_LENGTH / 6) << 8) / 209,
((SINE_TABLE_LENGTH / 6) << 8) / 210,
((SINE_TABLE_LENGTH / 6) << 8) / 211,
((SINE_TABLE_LENGTH / 6) << 8) / 212,
((SINE_TABLE_LENGTH / 6) << 8) / 213,
((SINE_TABLE_LENGTH / 6) << 8) / 214,
((SINE_TABLE_LENGTH / 6) << 8) / 215,
((SINE_TABLE_LENGTH / 6) << 8) / 216,
((SINE_TABLE_LENGTH / 6) << 8) / 217,
((SINE_TABLE_LENGTH / 6) << 8) / 218,
((SINE_TABLE_LENGTH / 6) << 8) / 219,
((SINE_TABLE_LENGTH / 6) << 8) / 220,
((SINE_TABLE_LENGTH / 6) << 8) / 221,
((SINE_TABLE_LENGTH / 6) << 8) / 222,
((SINE_TABLE_LENGTH / 6) << 8) / 223,
((SINE_TABLE_LENGTH / 6) << 8) / 224,
((SINE_TABLE_LENGTH / 6) << 8) / 225,
((SINE_TABLE_LENGTH / 6) << 8) / 226,
((SINE_TABLE_LENGTH / 6) << 8) / 227,
((SINE_TABLE_LENGTH / 6) << 8) / 228,
((SINE_TABLE_LENGTH / 6) << 8) / 229,
((SINE_TABLE_LENGTH / 6) << 8) / 230,
((SINE_TABLE_LENGTH / 6) << 8) / 231,
((SINE_TABLE_LENGTH / 6) << 8) / 232,
((SINE_TABLE_LENGTH / 6) << 8) / 233,
((SINE_TABLE_LENGTH / 6) << 8) / 234,
((SINE_TABLE_LENGTH / 6) << 8) / 235,
((SINE_TABLE_LENGTH / 6) << 8) / 236,
((SINE_TABLE_LENGTH / 6) << 8) / 237,
((SINE_TABLE_LENGTH / 6) << 8) / 238,
((SINE_TABLE_LENGTH / 6) << 8) / 239,
((SINE_TABLE_LENGTH / 6) << 8) / 240,
((SINE_TABLE_LENGTH / 6) << 8) / 241,
((SINE_TABLE_LENGTH / 6) << 8) / 242,
((SINE_TABLE_LENGTH / 6) << 8) / 243,
((SINE_TABLE_LENGTH / 6) << 8) / 244,
((SINE_TABLE_LENGTH / 6) << 8) / 245,
((SINE_TABLE_LENGTH / 6) << 8) / 246,
((SINE_TABLE_LENGTH / 6) << 8) / 247,
((SINE_TABLE_LENGTH / 6) << 8) / 248,
((SINE_TABLE_LENGTH / 6) << 8) / 249,
((SINE_TABLE_LENGTH / 6) << 8) / 250,
((SINE_TABLE_LENGTH / 6) << 8) / 251,
((SINE_TABLE_LENGTH / 6) << 8) / 252,
((SINE_TABLE_LENGTH / 6) << 8) / 253,
((SINE_TABLE_LENGTH / 6) << 8) / 254,
((SINE_TABLE_LENGTH / 6) << 8) / 255
};

/*! \brief Block commutation port direction masks, forward driving.
 *
 *  This array contains port direction masks for block commutation
 *  when running in the forward direction.
 */
__flash const uint8_t blockCommutationTableForward[16] =
{
  0,                          0,
  (1 << PB3),                 (1 << PD5),                 // UL, WH
  (1 << PB2),                 (1 << PD6),                 // UH, VL
  ((1 << PB2) | (1 << PB3)),  0x00,                       // VL, WH
  (1 << PB1),                 (1 << PD3),                 // VH, WL
  (1 << PB1),                 (1 << PD5),                 // UL, VH
  0x00,                       ((1 << PD6) | (1 << PD3)),  // UH, WL
  0,  0
};


/*! \brief Block commutation port direction masks, reverse driving.
 *
 *  This array contains port direction masks for block commutation
 *  when running in the reverse direction.
 */
__flash const uint8_t blockCommutationTableReverse[16] =
{
  0,  0,
  0x00,                       ((1 << PD6) | (1 << PD3)),  // UH, WL
  (1 << PB1),                 (1 << PD5),                 // UL, VH
  (1 << PB1),                 (1 << PD3),                 // VH, WL
  ((1 << PB2) | (1 << PB3)),  0x00,                       // VL, WH
  (1 << PB2),                 (1 << PD6),                 // UH, VL
  (1 << PB3),                 (1 << PD5),                 // UL, WH
  0,  0
};


/*! \brief Table of next expected hall sensor value when running forward.
 *
 *  This array contains the next expected hall sensor value when
 *  running in the forward direction. The value at the index pointed
 *  to by the current hall sensor value is the next expected hall
 *  sensor value in the forward direction.
 */
__flash const uint8_t expectedHallSequenceForward[7] =
{
  0xff,    3,    6,    2,    5,    1,    4
};



/*! \brief Table of next expected hall sensor value when running in the reverse direction.
 *
 *  This array contains the next expected hall sensor value when
 *  running in the reverse direction. The value at the index pointed
 *  to by the current hall sensor value is the next expected hall
 *  sensor value in the reverse direction.
 */
__flash const uint8_t expectedHallSequenceReverse[7] =
{
  0xff,    5,    3,    1,    6,    4,    2
};


/*! \brief Sine table offset values corresponding to hall sensor values when running in the forward direction.
 *
 *  This array contains the sine table offset that corresponds to
 *  the hall sensor values when running in the forward direction.
 *  At the moment of a hall change, using the new hall sensor value
 *  as index into this table returns the sine table offset that will
 *  synchronize the generated sine waves to the back-EMF of the motor.
 */
__flash const uint16_t CSOffsetsForward[8] =
{
  0,
  5 * (SINE_TABLE_LENGTH / 6),
  1 * (SINE_TABLE_LENGTH / 6),
  0 * (SINE_TABLE_LENGTH / 6),
  3 * (SINE_TABLE_LENGTH / 6),
  4 * (SINE_TABLE_LENGTH / 6),
  2 * (SINE_TABLE_LENGTH / 6),
  0
};


/*! \brief Sine table offset values corresponding to hall sensor values when running in the reverse direction.
 *
 *  This array contains the sine table offset that corresponds to
 *  the hall sensor values when running in the reverse direction.
 *  At the moment of a hall change, using the new hall sensor value
 *  as index into this table returns the sine table offset that will
 *  synchronize the generated sine waves to the back-EMF of the motor.
 */
__flash const uint16_t CSOffsetsReverse[8] =
{
  0,
  1 * (SINE_TABLE_LENGTH / 6),
  5 * (SINE_TABLE_LENGTH / 6),
  0 * (SINE_TABLE_LENGTH / 6),
  3 * (SINE_TABLE_LENGTH / 6),
  2 * (SINE_TABLE_LENGTH / 6),
  4 * (SINE_TABLE_LENGTH / 6),
  0
};


#endif
