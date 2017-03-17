#include <chrono>
#include <memory>
#include <thread>
#include <utility>

// OSVR Includes
#include <osvr/PluginKit/EyeTrackerInterfaceC.h>
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/Util/Logger.h>
// Fove Includes
#include "IFVRHeadset.h"

// Import the json config files
#include "com_osvr_fove_eyetracker_json.h"
#include "com_osvr_fove_tracker_json.h"

namespace {
OSVR_MessageType fovePluginMessage;

class HeadTrackerDevice {
  public:
    HeadTrackerDevice(OSVR_PluginRegContext ctx,
                      std::shared_ptr<Fove::IFVRHeadset> hset) {
        // Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        // Configure Tracker
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        // Create the device token with the options
        m_dev.initAsync(ctx, "HeadTracker", opts);

        // Send JSON descriptor
        m_dev.sendJsonDescriptor(com_osvr_fove_tracker_json);

        // Register update callback
        m_dev.registerUpdateCallback(this);

        // Get the a copy of headset pointer
		m_headset = hset; 
		
		// make logger
		m_logger = osvr::util::log::make_logger("OSVR-FOVE");
    }

    OSVR_ReturnCode update() {

        // Ensure headset is valid
        if (!m_headset) {
			m_logger->error("Headset is null, return from headTrack update");
            return OSVR_RETURN_FAILURE;
        }

        // Get pose
        const Fove::SFVR_Pose pose = m_headset->GetHMDPose();

        OSVR_TimeValue timestamp;
        osvrTimeValueGetNow(&timestamp);
        // Send the pose to OSVR
        OSVR_PoseState outPose;
        osvrPose3SetIdentity(&outPose);

		osvrQuatSetW(&outPose.rotation, -pose.orientation.w);
		osvrQuatSetX(&outPose.rotation, pose.orientation.x);
		osvrQuatSetY(&outPose.rotation, pose.orientation.y);
		osvrQuatSetZ(&outPose.rotation, -pose.orientation.z);

		osvrVec3SetX(&outPose.translation, pose.position.x);
		osvrVec3SetY(&outPose.translation, pose.position.y);
		osvrVec3SetZ(&outPose.translation, -pose.position.z); 

        osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &outPose, 0,
                                             &timestamp);

        return OSVR_RETURN_SUCCESS;
    }

  private:
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    std::shared_ptr<Fove::IFVRHeadset> m_headset;
	osvr::util::log::LoggerPtr m_logger;
};

class EyeTrackerDevice {
  public:
    EyeTrackerDevice(OSVR_PluginRegContext ctx,
                     std::shared_ptr<Fove::IFVRHeadset> hset) {
        // Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        // Configure Tracker
        osvrDeviceEyeTrackerConfigure(opts, &m_eyetracker, 2);

        // Create the device token with the options
        m_dev.initAsync(ctx, "EyeTracker", opts);

        // Send JSON descriptor
        m_dev.sendJsonDescriptor(com_osvr_fove_eyetracker_json);

        // Register update callback
        m_dev.registerUpdateCallback(this);

        // Get the a copy of headset pointer
		m_headset = hset;

		// make logger
		m_logger = osvr::util::log::make_logger("OSVR-FOVE");
    }

    OSVR_ReturnCode update() {

        // Ensure headset is valid
        if (!m_headset) {
			m_logger->error("Headset is null, return from eyeTrack update");
            return OSVR_RETURN_FAILURE;
        }

        // Get the gaze vectors
        const Fove::SFVR_Vec3 left =
            m_headset->GetGazeVector(Fove::EFVR_Eye::Left).vector;
        const Fove::SFVR_Vec3 right =
            m_headset->GetGazeVector(Fove::EFVR_Eye::Right).vector;

        // Get current time
        OSVR_TimeValue currentTime;
        osvrTimeValueGetNow(&currentTime);

        // Report gaze to OSVR
        // Currently this is only doing the direction
        OSVR_EyeGazeDirectionState leftGaze;
        OSVR_EyeGazeDirectionState rightGaze;
        osvrVec3SetX(&leftGaze, left.x);
        osvrVec3SetY(&leftGaze, left.y);
        osvrVec3SetZ(&leftGaze, left.z);
        osvrVec3SetX(&rightGaze, left.x);
        osvrVec3SetY(&rightGaze, left.y);
        osvrVec3SetZ(&rightGaze, left.z);
        osvrDeviceEyeTrackerReport3DGazeDirection(m_eyetracker, leftGaze, 0,
                                                  &currentTime);
        osvrDeviceEyeTrackerReport3DGazeDirection(m_eyetracker, rightGaze, 1,
                                                  &currentTime);

        return OSVR_RETURN_SUCCESS;
    }

  private:
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_EyeTrackerDeviceInterface m_eyetracker;
	std::shared_ptr<Fove::IFVRHeadset> m_headset;
	osvr::util::log::LoggerPtr m_logger;

}; // class EyeTrackerDevice

class HardwareDetection {
  public:
	HardwareDetection() : m_found(false), m_logger(osvr::util::log::make_logger("OSVR-FOVE")) {}

    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
        // if device was already detected, no need to go thru hardware detection
        // again
        if (m_found) {
            return OSVR_RETURN_SUCCESS;
        }
        // create headset
        m_headset.reset(Fove::GetFVRHeadset());
        if (!m_headset) {
			m_logger->error("Unable to create headset");
            return OSVR_RETURN_FAILURE;
        }

        // Request capabilities
        // Todo: This enables everything - can we know from OSVR what a client
        // wants?
        m_headset->Initialise(Fove::EFVR_ClientCapabilities::Position |
                              Fove::EFVR_ClientCapabilities::Orientation |
                              Fove::EFVR_ClientCapabilities::Gaze);

        // Wait for FOVE hardware to be connected
        int count = 0;
        int maxcount = 5;
        while (!m_headset->IsHardwareConnected() && count < maxcount) {
			m_logger->info("Waiting for hardware to be connected");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            count++;
        }
        if (count == maxcount) {
			m_logger->error("Hardware not connecting after 5 try");
            return OSVR_RETURN_FAILURE;
        }
		m_logger->info("Hardware detected");
        m_found = true;

        // create head traker and eye traker
        osvr::pluginkit::registerObjectForDeletion(
            ctx, new HeadTrackerDevice(ctx, m_headset));
        osvr::pluginkit::registerObjectForDeletion(
            ctx, new EyeTrackerDevice(ctx, m_headset));

        return OSVR_RETURN_SUCCESS;
    }

  private:
    bool m_found;
	std::shared_ptr<Fove::IFVRHeadset> m_headset;
	osvr::util::log::LoggerPtr m_logger;
};
} // namespace

OSVR_PLUGIN(com_osvr_fove) {

    osvrDeviceRegisterMessageType(ctx, "FovePluginMessage", &fovePluginMessage);

    osvr::pluginkit::PluginContext context(ctx);

    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}