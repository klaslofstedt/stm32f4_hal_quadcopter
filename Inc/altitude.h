#ifndef ALTITUDE_H
#define ALTITUDE_H

typedef struct{
    float altitude;
    float velocity;
    float dt;
} Altitude_t;

void AltitudeStartTask(void const * argument);
void TelemetryStartTask2(void const * argument);

#endif