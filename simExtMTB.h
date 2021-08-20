#pragma once

#ifdef _WIN32
    #define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif
#if defined (__linux) || defined (__APPLE__)
    #define SIM_DLLEXPORT extern "C"
#endif


// The 3 required entry points of the CoppeliaSim plugin:
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);
