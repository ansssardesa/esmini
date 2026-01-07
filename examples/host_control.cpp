/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp"


// Try to attach external controller to an existing object and activate domains.
// Returns true on success, false otherwise.
static bool AttachExternalControllerToObject(int objectId, bool activateLat, bool activateLong)
{
    int resultCode = SE_AssignExternalController(objectId, activateLat ? 1 : 0, activateLong ? 1 : 0);
    if (resultCode != 0) {
        std::printf("SE_AssignExternalController failed for object %d (rc=%d)\n", objectId, resultCode);
        return false;
    }

    std::printf("Attached ExternalController to object %d (lat=%d long=%d)\n", objectId, activateLat ? 1 : 0, activateLong ? 1 : 0);
    return true;
}

static int parseIntArg(const char *arg, int defaultVal)
{
    if (!arg) return defaultVal;
    char *end = nullptr;
    long v = strtol(arg, &end, 10);
    return (end && *end == '\0') ? static_cast<int>(v) : defaultVal;
}

int main(int argc, char **argv)
{
    const char *oscPath = nullptr;
    const char *modeArg = "both";
    int useViewer = 1;

    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--osc") == 0 && i + 1 < argc)
            oscPath = argv[++i];
        else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc)
            modeArg = argv[++i];
        else if (strcmp(argv[i], "--viewer") == 0 && i + 1 < argc)
            useViewer = parseIntArg(argv[++i], 1);
        else if (strcmp(argv[i], "--help") == 0)
        {
            std::printf("Usage: host_control --osc <scenario.xosc> --mode long|lat|both --viewer 0|1\n");
            return 0;
        }
    }

    if (!oscPath)
    {
        std::printf("Error: --osc <scenario.xosc> is required.\n");
        return 1;
    }

    // Init scenario: disable_ctrls=0, use_viewer per flag, threads=0, record=0
    if (SE_Init(oscPath, /*disable_ctrls*/ 0, useViewer, /*threads*/ 0, /*record*/ 0) != 0)
    {
        std::printf("SE_Init failed.\n");
        return 1;
    }

    const int egoId = 0; // default ego id; adapt if needed

    bool controlLong = false;
    bool controlLat  = false;
    if (std::strcmp(modeArg, "long") == 0) controlLong = true;
    else if (std::strcmp(modeArg, "lat") == 0) controlLat = true;
    else { controlLong = true; controlLat = true; }

    // Attach ExternalController to ego and activate domains based on mode
    (void)AttachExternalControllerToObject(egoId, /*activateLat*/ controlLat, /*activateLong*/ controlLong);

    std::printf("Starting host control: mode=%s viewer=%d\n", modeArg, useViewer);

    float t = 0.0f;
    const float dt = 0.05f; // 20 Hz
    const float maxSpd = 15.0f;

    for (int step = 0; step < 2000; ++step)
    {
        SE_ScenarioObjectState st;
        (void)SE_GetScenarioObjectState(egoId, &st);

        if (controlLong)
        {
            // ramp to 15 m/s over ~10s, then hold
            float targetSpeed = (t < 10.0f) ? (maxSpd * (t / 10.0f)) : maxSpd;
            SE_ReportObjectSpeed(egoId, targetSpeed);
        }

        if (controlLat)
        {
            // gentle lateral sine offset; use road t directly (alternatively lane+offset if you know the lane)
            float laneOffset = 0.5f * std::sin(t * 0.5f);
            SE_ReportObjectLateralPosition(egoId, st.t + laneOffset);
            // Or: SE_ReportObjectLateralLanePosition(egoId, st.laneId, laneOffset);
        }

        SE_StepDT(dt);
        t += dt;
    }

    std::printf("Host control finished.\n");
    return 0;
}
