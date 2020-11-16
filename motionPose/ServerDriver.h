#pragma once

#include "openvr_driver.h"
//#include "WatchdogProvider.h"
#include "openvr_math.h"
#include "MotionPoseController.h"

class CServerDriver_MotionPose : public vr::IServerTrackedDeviceProvider
{
public:
	virtual vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual void Cleanup();
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}

private:
	driver::CMotionPoseControllerDriver* m_pController = nullptr;
};