#include "ServerDriver.h"

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
vr::EVRInitError CServerDriver_MotionPose::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	m_pController = new driver::CMotionPoseControllerDriver();
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

	return vr::VRInitError_None;
}

void CServerDriver_MotionPose::Cleanup()
{
	delete m_pController;
	m_pController = NULL;
}

void CServerDriver_MotionPose::RunFrame()
{
	if (m_pController)
	{
		m_pController->RunFrame();
	}
}