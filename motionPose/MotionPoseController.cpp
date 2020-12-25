#include "MotionPoseController.h"
#include "third-party/easylogging++/easylogging++.h"

namespace driver
{
	CMotionPoseControllerDriver::CMotionPoseControllerDriver()
	{
		LOG(TRACE) << "CMotionPoseControllerDriver()";

		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		m_sSerialNumber = "MotionPoseVirtualController 0.2.2";
		m_sModelNumber = "MotionPoseVirtualController";

		rigYawOffset = 0;

		_pose.poseTimeOffset = 0;
		_pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
		_pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
		_pose.qRotation = { 1, 0, 0, 0 };
		_pose.vecDriverFromHeadTranslation[0] = 0;
		_pose.vecDriverFromHeadTranslation[1] = 0;
		_pose.vecDriverFromHeadTranslation[2] = 0;
		_pose.vecWorldFromDriverTranslation[0] = 0;
		_pose.vecWorldFromDriverTranslation[1] = 0;
		_pose.vecWorldFromDriverTranslation[2] = 0;
		_pose.vecPosition[0] = 0;
		_pose.vecPosition[1] = 0;
		_pose.vecPosition[2] = 0;

		rigOffset = { 0, 0, 0 };
	}

	CMotionPoseControllerDriver::~CMotionPoseControllerDriver()
	{
		LOG(TRACE) << "~CMotionPoseControllerDriver()";
	}

	vr::EVRInitError CMotionPoseControllerDriver::Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		LOG(INFO) << "Activate motionPoseController";

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

	bool CMotionPoseControllerDriver::openMmf(HANDLE& MapFile, char*& mmfFile, LPCWSTR szName, int BufferSize, bool& Connected)
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

		LOG(INFO) << "Successfully connected to " << szName;

		return true;
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

	// Create a string with last error message
	std::string CMotionPoseControllerDriver::GetLastErrorStdStr()
	{
		DWORD error = GetLastError();
		if (error)
		{
			LPVOID lpMsgBuf;
			DWORD bufLen = FormatMessage(
				FORMAT_MESSAGE_ALLOCATE_BUFFER |
				FORMAT_MESSAGE_FROM_SYSTEM |
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				error,
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
				(LPTSTR)&lpMsgBuf,
				0, NULL);
			if (bufLen)
			{
				LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
				std::string result(lpMsgStr, lpMsgStr + bufLen);

				LocalFree(lpMsgBuf);

				return result;
			}
		}
		return std::string();
	}

	vr::DriverPose_t CMotionPoseControllerDriver::GetPose()
	{
		__try
		{
			_pose.poseIsValid = false;
			_pose.deviceIsConnected = _moverConnected;
			_pose.shouldApplyHeadModel = false;
			_pose.willDriftInYaw = false;

			// Create connection to OVRMC
			if (!_ovrmcConnected)
			{
				if (openMmf(MapFile_OVRMC, mmfFile_OVRMC, L"Local\\OVRMC_MMFv1", 4096, _ovrmcConnected))
				{
					Data_OVRMC = (MMFstruct_OVRMC_v1*)mmfFile_OVRMC;					
				}			
			}
			else if (Data_OVRMC != nullptr)
			{
				// Get data from OVRMC
				rigOffset = Data_OVRMC->Translation;
				rigYawOffset = Data_OVRMC->Rotation.v[1];
			}

			if (!_moverConnected)
			{
				_pose.result = vr::TrackingResult_Calibrating_InProgress;
				_pose.qRotation = vrmath::quaternionFromYawRollPitch(rigYawOffset, 0, 0);

				// Init was unsuccessful, return empty pose
				if (!openMmf(MapFile_Mover, mmfFile_Mover, L"Local\\motionRigPose", 4096, _moverConnected))
				{
					return _pose;
				}

				rigPose = (MMFstruct_Mover_v1*)mmfFile_Mover;
			}

			if (rigPose != nullptr)
			{
				// Convert from mm to m
				vr::HmdVector3d_t rigTranslation = { 0 };
				rigTranslation.v[0] = rigPose->rigSway / (double)1000.0;
				rigTranslation.v[1] = rigPose->rigHeave / (double)1000.0;
				rigTranslation.v[2] = -rigPose->rigSurge / (double)1000.0;

				// Create the rotation. Convert from degree to radian
				_pose.qRotation = vrmath::quaternionFromYawRollPitch(rigYawOffset + rigPose->rigYaw * 0.01745329251994329, -rigPose->rigRoll * 0.01745329251994329, rigPose->rigPitch * 0.01745329251994329);

				// Create the translation (XYZ position in 3d space)
				vr::HmdVector3d_t rigVector = vrmath::quaternionRotateVector(_pose.qRotation, rigTranslation) + rigOffset;
				_pose.vecPosition[0] = rigVector.v[0];
				_pose.vecPosition[1] = rigVector.v[1];
				_pose.vecPosition[2] = rigVector.v[2];

				_pose.poseIsValid = true;
				_pose.result = vr::TrackingResult_Running_OK;
			}

			return _pose;
		}
		__except (GetExceptionCode() == EXCEPTION_IN_PAGE_ERROR ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
		{
			throw std::exception();
		}
	}

	void CMotionPoseControllerDriver::RunFrame()
	{
		vr::DriverPose_t newPose;

		try
		{
			newPose = GetPose();
		}
		catch (std::exception& e)
		{			
			LOG(ERROR) << "MMF failed: " << e.what() << " " << GetLastErrorStdStr();
			

			// Close mover connection
			if (CloseHandle(MapFile_Mover) == 0)
			{
				LOG(ERROR) << "Failed to close Mover handle! Error: " << GetLastError();
			}

			// Close ovrmc connection
			if (CloseHandle(MapFile_OVRMC) == 0)
			{
				LOG(ERROR) << "Failed to close OVRMC handle! Error: " << GetLastError();
			}

			_moverConnected = false;
			_ovrmcConnected = false;

			// Declare pose as invalid
			newPose.poseIsValid = false;
			newPose.result = vr::TrackingResult_Calibrating_InProgress;
		}

		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, newPose, sizeof(vr::DriverPose_t));

	}
};