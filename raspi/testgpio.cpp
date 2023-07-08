#include <iostream> // Include all needed libraries here
#include <wiringPi.h>

using namespace std; // No need to keep using “std”

int main()
{
    wiringPiSetup();    // Setup the library
    pinMode(25, INPUT); // Configure GPIO1 as an input
    pinMode(24, INPUT);

    // Main program loop
    while (1)
    {
        if (digitalRead(25) == 1)
        {
            printf("program 3\n");
        }
        else if (digitalRead(24) == 1)
        {
            printf("program 1\n");
        }
        else
        {
            printf("program 2\n");
        }
    }
    return 0;
}