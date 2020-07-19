#include "WatchdogProvider.h"



// driver namespace
namespace vrmotioncompensation
{
	namespace driver
	{
		vr::EVRInitError WatchdogProvider::Init(vr::IVRDriverContext* pDriverContext)
		{
			VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
			return vr::VRInitError_None;
		}

		void WatchdogProvider::Cleanup()
		{
			VR_CLEANUP_WATCHDOG_DRIVER_CONTEXT();
		}
	}
}