#ifndef COMMANDER_H
#define COMMANDER_H

#include <stdio.h>

typedef enum
{
    CMD_MAINTAIN,
    TAKE_OFF,
    GOTO,
    MOVE,
    LAND,
    RESET,
    CALIBRATE
} commands_enum;

typedef struct
{
    uint32_t cmds;
    float coordinate[3];
} commander_t;

extern uint8_t cmdr_update;

void commander_land(void);
void command_executor(commander_t *commander);
void commander_cradle(float dt);

#endif