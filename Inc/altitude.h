#ifndef ALTITUDE_H
#define ALTITUDE_H

typedef struct{
    float altitude;
    float velocity;
    float dt;
} Altitude_t;

void Altitude_StartTask(void const * argument);
void Telemetry_StartTask2(void const * argument);

#endif