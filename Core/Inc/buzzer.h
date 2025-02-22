// buzzer.h
#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

// Note frequencies
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define REST     0

//// Note durations in ms
//#define W  1000  // Whole note
//#define H  500   // Half note
//#define Q  250   // Quarter note
//#define E  125   // Eighth note
//#define S  62    // Sixteenth note
//#define DH 750   // Dotted half note
//#define DQ 375   // Dotted quarter note

#define W  300   // Whole note
#define H  150   // Half note
#define Q  75    // Quarter note
#define E  38    // Eighth note
#define S  19    // Sixteenth note
#define DH 225   // Dotted half note (1.5 × H = 150 × 1.5)
#define DQ 113   // Dotted quarter note (1.5 × Q = 75 × 1.5)

#define TEMPO_MULTIPLIER 2  // Makes everything 20% faster

void buzzer_init(void);
void play_note(uint16_t frequency, uint16_t duration);
void update_mario_melody(void);
void start_mario_melody(void);
void start_mario_death(void);
uint8_t is_melody_playing(void);

#endif // BUZZER_H
