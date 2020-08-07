#include "MotionPoseController.h"
#include "third-party/easylogging++/easylogging++.h"

INITIALIZE_EASYLOGGINGPP

namespace driver
{
	CMotionPoseControllerDriver::CMotionPoseControllerDriver()
	{
		init_logging();		

		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		m_sSerialNumber = "MotionPoseVirtualController 0.2.0";
		m_sModelNumber = "MotionPoseVirtualController";

		rigYawOffset = 0;

		pose.poseTimeOffset = 0;
		pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
		pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
		pose.qRotation = { 1, 0, 0, 0 };
		pose.vecDriverFromHeadTranslation[0] = 0;
		pose.vecDriverFromHeadTranslation[1] = 0;
		pose.vecDriverFromHeadTranslation[2] = 0;
		pose.vecWorldFromDriverTranslation[0] = 0;
		pose.vecWorldFromDriverTranslation[1] = 0;
		pose.vecWorldFromDriverTranslation[2] = 0;
		pose.vecPosition[0] = 0;
		pose.vecPosition[1] = 0;
		pose.vecPosition[2] = 0;

		rigOffset = { 0, 0, 0 };
	}

	CMotionPoseControllerDriver::~CMotionPoseControllerDriver()
	{
	}

	#define BUF_SIZE 256

	vr::EVRInitError CMotionPoseControllerDriver::Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		LOG(INFO) << "Activate motionPoseController\n";

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

	void CMotionPoseControllerDriver::init_logging()
	{
		el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
		el::Configurations conf(logConfigFileName);
		conf.parseFromText(logConfigDefault);
		//conf.parseFromFile(logConfigFileName);
		conf.setRemainingToDefault();
		el::Loggers::reconfigureAllLoggers(conf);

		LOG(INFO) << "|========================================================================================|";
		LOG(INFO) << "motionPose dll loaded...";
		LOG(TRACE) << "Trace messages enabled.";
		LOG(DEBUG) << "Debug messages enabled.";
	}

	bool CMotionPoseControllerDriver::openMmf(HANDLE& MapFile, char* mmfFile, LPCWSTR szName, int BufferSize, bool& Connected)
	{

		MapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, szName);

		// return if MMF could not be opened
		if (MapFile == NULL)
		{
			return false;
		}

		mmfFile = (char*)MapViewOfFile(MapFile,	FILE_MAP_READ, 0, 0, BufferSize);

		// return if view could not be mapped
		if (mmfFile == NULL)
		{
			CloseHandle(MapFile);
			return false;
		}

		Connected = true;

		return true;
	}

	bool CMotionPoseControllerDriver::initializeMoverMmf()
	{
		TCHAR szName[] = TEXT("Local\\motionRigPose");

		MapFile_Mover = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read / write access
										FALSE,                 // do not inherit the name
										szName);               // name of mapping object

		// return if MMF could not be opened
		if (MapFile_Mover == NULL)
		{
			return false;
		}

		mmfFile_Mover = (char*)MapViewOfFile(MapFile_Mover,				// handle to map object
											 FILE_MAP_READ,		// read / write permission
											 0,
											 0,
											 BUF_SIZE);

		// return if view could not be mapped
		if (mmfFile_Mover == NULL)
		{
			CloseHandle(MapFile_Mover);
			return false;
		}
		moverConnected = true;

		return true;
	}

	void CMotionPoseControllerDriver::initializeOvrmcMmf()
	{
		TCHAR szName[] = TEXT("Local\\OVRMC_MMFv1");

		MapFile_OVRMC = OpenFileMapping(FILE_MAP_ALL_ACCESS,   // read / write access
										FALSE,                 // do not inherit the name
										szName);               // name of mapping object

		if (MapFile_OVRMC == NULL)
		{
			return;
		}

		mmfFile_OVRMC = (char*)MapViewOfFile(MapFile_OVRMC,				// handle to map object
											 FILE_MAP_ALL_ACCESS,		// read / write permission
											 0,
											 0,
											 4096);

		if (mmfFile_Mover == NULL)
		{
			CloseHandle(MapFile_OVRMC);
			return;
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
		pose.poseIsValid = false;
		pose.deviceIsConnected = moverConnected;
		pose.shouldApplyHeadModel = false;
		pose.willDriftInYaw = false;

		vr::HmdQuaternion_t yaw = vrmath::quaternionFromRotationY(rigYawOffset);

		// Create connection to OVRMC
		if (!ovrmcConnected)
		{
			if (openMmf(MapFile_OVRMC, mmfFile_OVRMC, L"Local\\OVRMC_MMFv1", 4096, ovrmcConnected))
			{
				// try .. except block catches any error that occurs while trying to read the mmf
				__try
				{
					Data_OVRMC = (MMFstruct_OVRMC_v1*)mmfFile_OVRMC;
				}
				__except (GetExceptionCode() == EXCEPTION_IN_PAGE_ERROR ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
				{
					LOG(ERROR) << "Failed assign pointer to OVRMC mmf!";

					// Close connection
					if (CloseHandle(MapFile_OVRMC) == 0)
					{
						LOG(ERROR) << "Failed to close OVRMC handle! - 1 -  Error: " << GetLastError();
					}

					ovrmcConnected = false;
				}
			}			
		}
		else
		{
			// try .. except block catches any error that occurs while trying to read the mmf
			__try
			{
				// Get data from OVRMC
				rigOffset = Data_OVRMC->Translation;
				rigYawOffset = Data_OVRMC->Rotation.v[1];
			}
			__except (GetExceptionCode() == EXCEPTION_IN_PAGE_ERROR ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
			{
				LOG(ERROR) << "Failed to read from OVRMC mmf!";

				// Close connection
				if (CloseHandle(MapFile_OVRMC) == 0)
				{
					LOG(ERROR) << "Failed to close OVRMC handle! - 2 - Error: " << GetLastError();
				}

				ovrmcConnected = false;
			}
		}

		/*// Turn Right/Left
		if ((GetAsyncKeyState('E') & 0x8000) != 0)			rigYawOffset += 0.005;
		if ((GetAsyncKeyState('R') & 0x8000) != 0)			rigYawOffset -= 0.005;

		// Offset Left/Right
		if ((GetAsyncKeyState('Q') & 0x8000) != 0)			rigOffset = rigOffset + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(-0.003, 0, 0));
		if ((GetAsyncKeyState('W') & 0x8000) != 0)			rigOffset = rigOffset + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0.003, 0, 0));

		// Offset Up/Down
		if ((GetAsyncKeyState('A') & 0x8000) != 0)			rigOffset = rigOffset + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, 0.003, 0));
		if ((GetAsyncKeyState('S') & 0x8000) != 0)			rigOffset = rigOffset + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, -0.003, 0));

		// Offset Forward/Backward
		if ((GetAsyncKeyState('Y') & 0x8000) != 0)			rigOffset = rigOffset + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, 0, -0.003));
		if ((GetAsyncKeyState('X') & 0x8000) != 0)			rigOffset = rigOffset + vrmath::quaternionRotateVector(yaw, HmdVector3d_t_Init(0, 0, 0.003));

		// reset
		if ((GetAsyncKeyState('P') & 0x8000) != 0)
		{
			rigOffset = { 0 };
			rigYawOffset = 0;
		}*/

		if (!moverConnected)
		{
			pose.result = vr::TrackingResult_Calibrating_InProgress;
			pose.qRotation = vrmath::quaternionFromYawRollPitch(rigYawOffset, 0, 0);

			// Init was unsuccessful, return empty pose
			if (!openMmf(MapFile_Mover, mmfFile_Mover, L"Local\\motionRigPose", 4096, moverConnected))
			{
				return pose;
			}

			// try .. except block catches any error that occurs while trying to read the mmf
			__try
			{
				rigPose = (MMFstruct_Mover_v1*)mmfFile_Mover;
			}
			__except (GetExceptionCode() == EXCEPTION_IN_PAGE_ERROR ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
			{
				LOG(ERROR) << "Failed to assign pointer to Mover mmf!";

				// Close connection
				if (CloseHandle(MapFile_Mover) == 0)
				{
					LOG(ERROR) << "Failed to close Mover handle! - 1 - Error: " << GetLastError();
				}

				moverConnected = false;
			}
		}

		// try .. except block catches any error that occurs while trying to read the mmf
		__try
		{
			// Convert from mm to m
			vr::HmdVector3d_t rigTranslation = { 0 };
			rigTranslation.v[0] = rigPose->rigSway / (double)1000.0;
			rigTranslation.v[1] = rigPose->rigHeave / (double)1000.0;
			rigTranslation.v[2] = -rigPose->rigSurge / (double)1000.0;

			// Create the rotation. Convert from degree to radian
			pose.qRotation = vrmath::quaternionFromYawRollPitch(rigYawOffset + rigPose->rigYaw * 0.01745329251994329, -rigPose->rigRoll * 0.01745329251994329, rigPose->rigPitch * 0.01745329251994329);

			// Create the translation (XYZ position in 3d space)
			vr::HmdVector3d_t rigVector = vrmath::quaternionRotateVector(pose.qRotation, rigTranslation) + rigOffset;
			pose.vecPosition[0] = rigVector.v[0];
			pose.vecPosition[1] = rigVector.v[1];
			pose.vecPosition[2] = rigVector.v[2];

			pose.poseIsValid = true;
			pose.result = vr::TrackingResult_Running_OK;
		}
		__except (GetExceptionCode() == EXCEPTION_IN_PAGE_ERROR ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
		{
			LOG(ERROR) << "Failed to read from Mover mmf!";

			// Close connection
			if (CloseHandle(MapFile_Mover) == 0)
			{
				LOG(ERROR) << "Failed to close Mover handle! - 2 - Error: " << GetLastError();
			}

			moverConnected = false;

			// Declare pose as invalid
			pose.poseIsValid = false;
			pose.result = vr::TrackingResult_Calibrating_InProgress;
		}

		return pose;
	}

	void CMotionPoseControllerDriver::RunFrame()
	{
		// Your driver would read whatever hardware state is associated with its input components and pass that
		// in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
		// state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.

		// Collect events
		/*vr::VREvent_t event;
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