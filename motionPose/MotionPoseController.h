#pragma once

#include <windows.h>
#include "openvr_driver.h"
#include "openvr_math.h"

namespace driver
{
	// struct for OVRMC MMF version 1
	struct MMFstruct_OVRMC_v1
	{
		vr::HmdVector3d_t Translation;
		vr::HmdVector3d_t Rotation;
		vr::HmdQuaternion_t QRotation;
		uint32_t Flags_1;
		uint32_t Flags_2;
		double Reserved_double[10];
		int Reserved_int[10];

		MMFstruct_OVRMC_v1()
		{
			Translation = { 0, 0, 0 };
			Rotation = { 0, 0, 0 };
			QRotation = { 0, 0, 0, 0 };
			Flags_1 = 0;
			Flags_2 = 0;
		}
	};

	// struct for FlyPT Mover version 1
	struct MMFstruct_Mover_v1
	{
		double rigSway;
		double rigSurge;
		double rigHeave;
		double rigYaw;
		double rigRoll;
		double rigPitch;
	};

	class CMotionPoseControllerDriver : public vr::ITrackedDeviceServerDriver
	{
		// Motion Rig pose as reported from FlyPT Mover
		MMFstruct_Mover_v1* rigPose;		

		// pose that is send to SteamVR
		vr::DriverPose_t pose;

		// Offset from zero-point in tracking space
		vr::HmdVector3d_t rigOffset;
		double rigYawOffset;

		// MMF: FlyPT Mover
		char* mmfFile_Mover = nullptr;
		HANDLE MapFile_Mover = NULL;
		bool moverConnected = false;

		// MMF: OVRMC
		char* mmfFile_OVRMC = nullptr;
		HANDLE MapFile_OVRMC = NULL;
		MMFstruct_OVRMC_v1* Data_OVRMC = nullptr;
		bool ovrmcConnected = false;

		// easylogging++ settings
		const char* logConfigFileName = "logging.conf";

		const char* logConfigDefault =
			"* GLOBAL:\n"
			"	FORMAT = \"[%level] %datetime{%Y-%M-%d %H:%m:%s}: %msg\"\n"
			"	FILENAME = \"driver_motioncompensation.log\"\n"
			"	ENABLED = true\n"
			"	TO_FILE = true\n"
			"	TO_STANDARD_OUTPUT = true\n"
			"	MAX_LOG_FILE_SIZE = 2097152 ## 2MB\n"
			"* TRACE:\n"
			"	ENABLED = false\n"
			"* DEBUG:\n"
			"	ENABLED = true\n";

	public:
		CMotionPoseControllerDriver();

		virtual ~CMotionPoseControllerDriver();

		inline vr::HmdVector3d_t HmdVector3d_t_Init(double x, double y, double z)
		{
			vr::HmdVector3d_t vec;
			vec.v[0] = x;
			vec.v[1] = y;
			vec.v[2] = z;
			return vec;
		}

		virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);

		void init_logging();

		bool initializeMoverMmf();

		void initializeOvrmcMmf();

		bool openMmf(HANDLE& MapFile, char* mmfFile, LPCWSTR szName, int BufferSize, bool& Connected);

		virtual void Deactivate();

		virtual void EnterStandby();

		void* GetComponent(const char* pchComponentNameAndVersion);

		virtual void PowerOff();

		/** debug request from a client */
		virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize);

		virtual vr::DriverPose_t GetPose();

		void RunFrame();

		std::string GetSerialNumber() const { return m_sSerialNumber; }

	private:
		vr::TrackedDeviceIndex_t m_unObjectId;
		vr::PropertyContainerHandle_t m_ulPropertyContainer;

		std::string m_sSerialNumber;
		std::string m_sModelNumber;
	};
}