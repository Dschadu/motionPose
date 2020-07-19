#include "driver_motionPose.h"

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_motionPoseDriver;
	}
	else if (std::strcmp(vr::IVRWatchdogProvider_Version, pInterfaceName) == 0)
	{
		return &watchdogProvider;
	}

	if (pReturnCode)
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

	return NULL;
}