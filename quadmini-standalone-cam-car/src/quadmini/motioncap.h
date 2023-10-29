#ifndef MOTIONCAP_H
#define MOTIONCAP_H

#include <stdio.h>

typedef struct
{
    float mc_pos[3];
    float mc_vel[3];
    float mc_euler[3];
    float mc_vel_calc[3];
} motioncap_t;

typedef enum {
    MC_X = 0,
    MC_Y,
    MC_Z
} mc_axis_enum;

extern motioncap_t motioncap_data;

float getMotioncapSpeed(mc_axis_enum axis);
float getMotioncapPosition(mc_axis_enum axis);
void speed_fusion_mc(float dt);
void height_fusion_20_mc(float dt);
void motioncap_on_pack_received(void *data);
void motioncap_cradle(float dt);

#endif
