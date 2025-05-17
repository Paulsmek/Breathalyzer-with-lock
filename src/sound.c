#include <stdint.h>

#include "sound.h"

/* Tones (Hz) */
#define c    261
#define d    294
#define e    329
#define f    349
#define g    391
#define gS   415
#define a    440
#define aS   455
#define b    466
#define cH   523
#define cSH  554
#define dH   587
#define dSH  622
#define eH   659
#define fH   698
#define fSH  740
#define gH   784
#define gSH  830
#define aH   880

/* The nodes in our melody */
uint16_t surprise_notes[] = {a,   a,  a,   f,  cH,  a,  f,   cH, a,   eH, eH,
                             eH,  fH, cH,  gS, f,   cH, a,   aH, a,   a,  aH,
                             gSH, gH, fSH, fH, fSH, aS, dSH, dH, cSH, cH, b,
                             cH,  f,  gS,  f,  a,   cH, a,   cH, eH,  aH, a,
                             a,   aH, gSH, gH, fSH, fH, fSH, aS, dSH, dH, cSH,
                             cH,  b,  cH,  f,  gS,  f,  cH,  a,  f,   cH, a};

/* The durations (in ms) of every note in the melody above */
uint16_t durations[] = {500, 500, 500, 350, 150, 500, 350, 150, 650, 500, 500,
                        500, 350, 150, 500, 350, 150, 650, 500, 300, 150, 400,
                        200, 200, 125, 125, 250, 250, 400, 200, 200, 125, 125,
                        250, 125, 500, 375, 125, 500, 375, 125, 650, 500, 300,
                        150, 400, 200, 200, 125, 125, 250, 250, 400, 200, 200,
                        125, 125, 250, 250, 500, 375, 125, 500, 375, 125, 650};

/* Total number of notes in melody (use for iterating!) */
uint8_t num_notes = sizeof(durations) / sizeof(durations[0]);

