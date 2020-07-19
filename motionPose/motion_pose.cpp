#include <openvr_driver.h>
#include "openvr_math.h"
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>
#include <math.h>

#include <stdio.h>
#include <stdarg.h>

#if defined( _WINDOWS )
#include <windows.h>
#endif

#include <conio.h>
#include <tchar.h>
#include "WatchdogProvider.h"

using namespace vr;


#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

inline HmdVector3d_t HmdVector3d_t_Init(double x, double y, double z)
{
	HmdVector3d_t vec;
	vec.v[0] = x;
	vec.v[1] = y;
	vec.v[2] = z;
	return vec;
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CMotionPoseControllerDriver : public vr::ITrackedDeviceServerDriver
{

	double rigX;
	double rigY;
	double rigZ;
	double rigPitch;
	double rigRoll;
	double rigYaw;
	double rigHeave;
	double rigSurge;
	double rigSway;
	double yawComp;

	char* mmfFile;
	DriverPose_t pose;
	int hmdDeviceId;
	bool hmdValid = false;
	bool moverConnected = false;
	TrackedDevicePose_t poses[10];
	HANDLE hMapFile = NULL;
	HmdVector3d_t rigPos;

public:
	CMotionPoseControllerDriver()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		m_sSerialNumber = "MotionPoseVirtualController 0.1";

		m_sModelNumber = "MotionPoseVirtualController";
		rigPitch = 0;
		rigRoll = 0;
		rigYaw = 0;
		rigHeave = 0;
		rigSurge = 0;
		rigSway = 0;
		yawComp = 0;
		pose = { 0 };
		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.vecDriverFromHeadTranslation[0] = 0;
		pose.vecDriverFromHeadTranslation[1] = 0;
		pose.vecDriverFromHeadTranslation[2] = 0;
		pose.vecWorldFromDriverTranslation[0] = 0;
		pose.vecWorldFromDriverTranslation[1] = 0;
		pose.vecWorldFromDriverTranslation[2] = 0;
		pose.vecPosition[0] = 0;
		pose.vecPosition[1] = 0;
		pose.vecPosition[2] = 0;
		rigPos = { 0 };
	}

	virtual ~CMotionPoseControllerDriver()
	{
	}

#define BUF_SIZE 256


	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		DriverLog("Activate motionPoseController\n");

		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "locator");

		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);

		return VRInitError_None;
	}

	void initalizeMoverMmf() {
		moverConnected = false;
		TCHAR szName[] = TEXT("Local\\motionRigPose");

		hMapFile = OpenFileMapping(
			FILE_MAP_ALL_ACCESS,   // read/write access
			FALSE,                 // do not inherit the name
			szName);               // name of mapping object

		if (hMapFile == NULL)
		{
			//			DriverLog("could not open mmf motionRigPose\n");
		}
		else {
			//			DriverLog("successfully opened mmf motionRigPose\n");
			mmfFile = (char*)MapViewOfFile(hMapFile, // handle to map object
				FILE_MAP_ALL_ACCESS,  // read/write permission
				0,
				0,
				BUF_SIZE);
			if (strncmp(mmfFile, "posedata", 8) == 0) {
				DriverLog("successfully receiving posedata\n");
				moverConnected = true;
			}
			else {
//				DriverLog("no valid posedata\n");
			}
		}
	}

	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	virtual DriverPose_t GetPose()
	{
		//		DriverLog("virtualPoseController::GetPose\n");

		pose.poseIsValid = true;
		pose.deviceIsConnected = true;
		pose.shouldApplyHeadModel = false;
		pose.willDriftInYaw = false;

		HmdQuaternion_t yaw = vrmath::quaternionFromRotationY(yawComp);

		// Turn Right/Left
		if ((GetAsyncKeyState('E') & 0x8000) != 0)			yawComp += 0.005;
		if ((GetAsyncKeyState('R') & 0x8000) != 0)			yawComp -= 0.005;

		// Offset Left/Right
		if ((GetAsyncKeyState('Q') & 0x8000) != 0)			rigPos = rigPos + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(-0.003, 0, 0));
		if ((GetAsyncKeyState('W') & 0x8000) != 0)			rigPos = rigPos + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0.003, 0, 0));

		// Offset Up/Down
		if ((GetAsyncKeyState('A') & 0x8000) != 0)			rigPos = rigPos + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, 0.003, 0));
		if ((GetAsyncKeyState('S') & 0x8000) != 0)			rigPos = rigPos + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, -0.003, 0));

		// Offset Forward/Backward
		if ((GetAsyncKeyState('Y') & 0x8000) != 0)			rigPos = rigPos + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, 0, -0.003));
		if ((GetAsyncKeyState('X') & 0x8000) != 0)			rigPos = rigPos + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, 0, 0.003));

		// reset
		if ((GetAsyncKeyState('P') & 0x8000) != 0) {
			rigPos = { 0 };
			yawComp = 0;
		}

		if (moverConnected && strncmp(mmfFile, "posedata", 8) == 0) {
			pose.result = TrackingResult_Running_OK;
			// <PoseSway><PoseSurge><PoseHeave><PoseYaw><PoseRoll><PosePitch>
			double* poseData = (double*)mmfFile;
			rigSway = poseData[1];
			rigSurge = poseData[2];
			rigHeave = poseData[3];
			rigYaw = poseData[4];
			rigRoll = poseData[5];
			rigPitch = poseData[6];
			/*			char buf[100];
						sprintf_s(buf, "virtualPoseController::rigYaw: %lf\n", rigYaw);
						DriverLog(buf);
						sprintf_s(buf, "virtualPoseController::rigPitch: %lf\n", rigPitch);
						DriverLog(buf);*/

			vr::HmdVector3d_t rigTranslation = { 0 };
			rigTranslation.v[0] = rigSway / 1000;
			rigTranslation.v[1] = rigHeave / 1000;
			rigTranslation.v[2] = -rigSurge / 1000;

			pose.qRotation = vrmath::quaternionFromYawRollPitch(yawComp + rigYaw * 0.01745329251994329, -rigRoll * 0.01745329251994329, rigPitch * 0.01745329251994329);
			vr::HmdVector3d_t rigVector = vrmath::quaternionRotateVector(pose.qRotation, rigTranslation) + rigPos;
			pose.vecPosition[0] = rigVector.v[0];
			pose.vecPosition[1] = rigVector.v[1];
			pose.vecPosition[2] = rigVector.v[2];

			/*			pose.qWorldFromDriverRotation = vrmath::quaternionFromYawPitchRoll(yawComp, 0, 0);
						pose.vecWorldFromDriverTranslation[0] = rigX;
						pose.vecWorldFromDriverTranslation[1] = rigY;
						pose.vecWorldFromDriverTranslation[2] = rigZ;*/

			return pose;
		}
		else {
			pose.result = TrackingResult_Calibrating_InProgress;
			pose.deviceIsConnected = false;
			initalizeMoverMmf();

			pose.qRotation = vrmath::quaternionFromYawRollPitch(yawComp, 0, 0);
			vr::HmdVector3d_t rigVector = rigPos;

			return pose;
		}

	}





	void RunFrame()
	{
		// Your driver would read whatever hardware state is associated with its input components and pass that
		// in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
		// state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.
		// DriverLog(" virtualPoseController::RunFrame\n");

		// Collect events
/*		vr::VREvent_t event;
		while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event)))
		{
			if (event.trackedDeviceIndex == this->m_unObjectId && event.eventType == EVREventType::VREvent_PropertyChanged) {
				if (event.data.property.prop == Prop_RenderModelName_String) {
					std::string renderModel = vr::VRProperties()->GetStringProperty(event.data.property.container, Prop_RenderModelName_String);
					vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, renderModel.data());						
				}
			}
		}*/

		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
	}

	std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;
	std::string m_sModelNumber;


};

class CServerDriver_MotionPose : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual void Cleanup();
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}

private:
	CMotionPoseControllerDriver* m_pController = nullptr;
};

CServerDriver_MotionPose g_motionPoseDriver;


EVRInitError CServerDriver_MotionPose::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());

	m_pController = new CMotionPoseControllerDriver();
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

	return VRInitError_None;
}

void CServerDriver_MotionPose::Cleanup()
{
	CleanupDriverLog();
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

vrmotioncompensation::driver::WatchdogProvider watchdogProvider;

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_motionPoseDriver;
	}
	else if (std::strcmp(IVRWatchdogProvider_Version, pInterfaceName) == 0)
	{
		return &watchdogProvider;
	}

	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
