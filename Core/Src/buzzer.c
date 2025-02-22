// buzzer.c
#include "buzzer.h"

extern TIM_HandleTypeDef htim5;

// Complete Mario theme melody
static const uint16_t mario_melody[] = {
    // Part 1
    NOTE_E5, NOTE_E5, REST, NOTE_E5, REST, NOTE_C5, NOTE_E5, REST,
    NOTE_G5, REST, REST, NOTE_G4, REST,

    // Part 2
    NOTE_C5, REST, REST, NOTE_G4, REST, NOTE_E4, REST,
    NOTE_A4, REST, NOTE_B4, REST, NOTE_AS4, NOTE_A4, REST,

    // Part 3
    NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, REST, NOTE_F5, NOTE_G5,
    REST, NOTE_E5, REST, NOTE_C5, NOTE_D5, NOTE_B4, REST,

    // Part 4 (Underground theme)
    NOTE_C5, REST, NOTE_G4, REST, NOTE_E4, REST,
    NOTE_A4, REST, NOTE_B4, REST, NOTE_A4, NOTE_GS4,
    NOTE_AS4, NOTE_GS4, NOTE_G4, NOTE_F4, NOTE_G4
};

static const uint16_t mario_durations[] = {
    // Part 1
    E, E, E, E, E, E, E, E,
    Q, E, Q, Q, H,

    // Part 2
    Q, E, Q, Q, Q, Q, Q,
    Q, E, Q, E, E, Q, E,

    // Part 3
    E, E, E, Q, E, E, Q,
    E, E, E, E, E, Q, Q,

    // Part 4
    Q, E, Q, E, Q, E,
    Q, E, Q, E, E, E,
    Q, E, E, E, H
};

// Mario death melody
static const uint16_t death_melody[] = {
    NOTE_C5, NOTE_CS5, NOTE_D5,
    REST,
    NOTE_B4, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5,
    REST,
    NOTE_E4, NOTE_E4, NOTE_C4
};

static const uint16_t death_durations[] = {
    E, E, E,
    E,
    E, E, E, E, E, E, Q,
    E,
    E, E, Q
};

// State variables for non-blocking playback
static uint32_t note_timer = 0;
static uint32_t note_index = 0;
static uint8_t melody_playing = 0;
static uint32_t note_duration = 0;
static uint32_t space_duration = 0;
static uint8_t note_state = 0;  // 0: Note off, 1: Note playing, 2: Space between notes
static uint8_t current_melody = 0;  // 0: Main theme, 1: Death sound

void buzzer_init(void)
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}

void start_mario_melody(void)
{
    note_index = 0;
    melody_playing = 1;
    note_timer = HAL_GetTick();
    note_state = 1;
    current_melody = 0;
    note_duration = mario_durations[0];
}

void start_mario_death(void)
{
    note_index = 0;
    melody_playing = 1;
    note_timer = HAL_GetTick();
    note_state = 1;
    current_melody = 1;
    note_duration = death_durations[0];
}

uint8_t is_melody_playing(void)
{
    return melody_playing;
}

void play_note(uint16_t frequency, uint16_t duration)
{
    if(frequency == REST)
    {
        HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
        return;
    }

    uint32_t period = (80000000 / ((91 + 1) * frequency)) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim5, period);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, period/2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}

void update_mario_melody(void)
{
    if (!melody_playing) return;

    uint32_t current_time = HAL_GetTick();
    const uint16_t *current_melody_array = current_melody ? death_melody : mario_melody;
    const uint16_t *current_duration_array = current_melody ? death_durations : mario_durations;
    const uint16_t melody_size = current_melody ?
        sizeof(death_melody)/sizeof(death_melody[0]) :
        sizeof(mario_melody)/sizeof(mario_melody[0]);

    switch(note_state) {
        case 1:  // Note is playing
            if (current_time - note_timer >= note_duration) {
                HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
                note_timer = current_time;
                space_duration = 20 * TEMPO_MULTIPLIER;  // Space between notes
                note_state = 2;
            }
            break;

        case 2:  // Space between notes
            if (current_time - note_timer >= space_duration) {
                note_index++;
                if (note_index >= melody_size) {
                    note_index = 0;  // Reset index instead of stopping
                }

                if (current_melody_array[note_index] != REST) {
                    play_note(current_melody_array[note_index], current_duration_array[note_index]);
                }

                note_duration = current_duration_array[note_index];
                note_timer = current_time;
                note_state = 1;
            }
            break;

        default:
            note_state = 1;
            break;
    }
}
