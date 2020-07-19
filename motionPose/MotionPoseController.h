#pragma once

#include <windows.h>
#include "openvr_driver.h"
#include "openvr_math.h"

namespace driver
{
	class CMotionPoseControllerDriver : public vr::ITrackedDeviceServerDriver
	{
		#define BUF_SIZE 256

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
		vr::DriverPose_t pose;
		int hmdDeviceId;
		bool hmdValid = false;
		bool moverConnected = false;
		vr::TrackedDevicePose_t poses[10];
		HANDLE hMapFile = NULL;
		vr::HmdVector3d_t rigPos;

	public:
		CMotionPoseControllerDriver();

		virtual ~CMotionPoseControllerDriver();

		inline vr::HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
		{
			vr::HmdQuaternion_t quat;
			quat.w = w;
			quat.x = x;
			quat.y = y;
			quat.z = z;
			return quat;
		}

		inline vr::HmdVector3d_t HmdVector3d_t_Init(double x, double y, double z)
		{
			vr::HmdVector3d_t vec;
			vec.v[0] = x;
			vec.v[1] = y;
			vec.v[2] = z;
			return vec;
		}

		virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);

		void initalizeMoverMmf();

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