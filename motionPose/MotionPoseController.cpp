#include "MotionPoseController.h"
#include "driverlog.h"

//class CMotionPoseControllerDriver : public vr::ITrackedDeviceServerDriver
namespace driver
{
	CMotionPoseControllerDriver::CMotionPoseControllerDriver()
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

	CMotionPoseControllerDriver::~CMotionPoseControllerDriver()
	{
	}

	#define BUF_SIZE 256

	vr::EVRInitError CMotionPoseControllerDriver::Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		DriverLog("Activate motionPoseController\n");

		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "locator");

		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, 2);

		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_OptOut);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);

		return vr::VRInitError_None;
	}

	void CMotionPoseControllerDriver::initalizeMoverMmf()
	{
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
		else
		{
			//			DriverLog("successfully opened mmf motionRigPose\n");
			mmfFile = (char*)MapViewOfFile(hMapFile,				// handle to map object
										   FILE_MAP_ALL_ACCESS,		// read/write permission
										   0,
										   0,
										   BUF_SIZE);
			if (strncmp(mmfFile, "posedata", 8) == 0)
			{
				DriverLog("successfully receiving posedata\n");
				moverConnected = true;
			}
			else
			{
				// DriverLog("no valid posedata\n");
			}
		}
	}

	void CMotionPoseControllerDriver::Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	void CMotionPoseControllerDriver::EnterStandby()
	{
	}

	void* CMotionPoseControllerDriver::GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	void CMotionPoseControllerDriver::PowerOff()
	{
	}

	/** debug request from a client */
	void CMotionPoseControllerDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	vr::DriverPose_t CMotionPoseControllerDriver::GetPose()
	{
		//		DriverLog("virtualPoseController::GetPose\n");

		pose.poseIsValid = true;
		pose.deviceIsConnected = true;
		pose.shouldApplyHeadModel = false;
		pose.willDriftInYaw = false;

		vr::HmdQuaternion_t yaw = vrmath::quaternionFromRotationY(yawComp);

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
		if ((GetAsyncKeyState('P') & 0x8000) != 0)
		{
			rigPos = { 0 };
			yawComp = 0;
		}

		if (moverConnected && strncmp(mmfFile, "posedata", 8) == 0)
		{
			pose.result = vr::TrackingResult_Running_OK;
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
		else
		{
			pose.result = vr::TrackingResult_Calibrating_InProgress;
			pose.deviceIsConnected = false;
			initalizeMoverMmf();

			pose.qRotation = vrmath::quaternionFromYawRollPitch(yawComp, 0, 0);
			vr::HmdVector3d_t rigVector = rigPos;

			return pose;
		}

	}

	void CMotionPoseControllerDriver::RunFrame()
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

		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(vr::DriverPose_t));
	}
};