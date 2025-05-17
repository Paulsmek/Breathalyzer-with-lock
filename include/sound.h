#ifndef SOUND_H_
#define SOUND_H_

#include <stdint.h>

/* The nodes in our melody */
extern uint16_t surprise_notes[];

/* The durations (in ms) of every note in the melody above */
extern uint16_t durations[];

/* Total number of notes in melody (use for iterating!) */
extern uint8_t num_notes;

#endif
