package frc.lib;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionHandler {
    public static final double NO_TARGET_FOUND = Double.MAX_VALUE;
    private final PhotonCamera photonCamera;
    private PhotonTrackedTarget lastResult = null;

    public PhotonVisionHandler(String cameraName) {
        this.photonCamera = new PhotonCamera(cameraName);
    }

    public double getTargetYaw() {
        // Read in relevant data from the Camera
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (PhotonTrackedTarget target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        lastResult = target;
                        return target.getYaw();
                    }
                }
            }
        }
        return NO_TARGET_FOUND;
    }

    public PhotonTrackedTarget getLastResult() {
        return lastResult;
    }
}