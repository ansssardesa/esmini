#include "esminiLib.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char* argv[])
{
    SE_InitWithArgs(argc, argv, "");
    // Acquire ego object id (adjust name if your scenario uses another)
    int egoId = 0;
    // If available in your build you can instead do:
    // egoId = SE_GetObjectIdFromName("Ego"); // fallback to 0 if not found

    for (int i = 0; i < 1000; i++)
    {
        // Manually control only ego BEFORE stepping
        {
            float x,y,z,h,p,r;
            SE_GetObjectPos(egoId,&x,&y,&z,&h,&p,&r);
            const float dt = 0.01f;          // 100 Hz
            const float speed = 10.0f;       // m/s forward
            x += speed * dt * cosf(h);
            y += speed * dt * sinf(h);
            SE_SetObjectPos(egoId,x,y,z,h,p,r);
            // Optionally: SE_SetObjectSpeed(egoId, speed);
        }

        SE_Step();
        // ...existing code...
        if (i % 100 == 0)
        {
            // ...existing code...
        }
    }
    // ...existing code...
}