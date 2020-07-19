#pragma once

/*
	We wont be using any cryptography, DDE, RPC, etc.. so exclude these from the build process, not necessary but speeds up the build a bit
*/
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "ServerDriver.h"
#include "WatchdogProvider.h"

/*
	Obviously we need to include the openvr_driver.h file so we can do the stuff
*/
#include <openvr_driver.h>
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport) 
using namespace vr;

/*
	Other helpful includes
*/
#include <vector>
#include <thread>
#include <chrono>

// Our driver factory function
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode);
CServerDriver_MotionPose g_motionPoseDriver;
vrmotioncompensation::driver::WatchdogProvider watchdogProvider;