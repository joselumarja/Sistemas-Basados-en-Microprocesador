#ifndef DEFINITIONS_H

#define TRUE 1
#define FALSE 0

struct nearbyCarsState
{
    int ReadyOperation;
    int CarsInProximities;
};
enum TrafficLightState
{
    WaitingInterrupt,
    CarGreen,
    ButtonPressed,
    CarYellow,
    CarRed,
    WalkerBlinkingGreen
};

#endif