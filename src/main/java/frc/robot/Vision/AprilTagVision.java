// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class AprilTagVision {
    PhotonCamera aprilTagsCamera;

    public AprilTagVision() {
        aprilTagsCamera = new PhotonCamera("AprilTags-Camera");
    }

    public int seesSpecificAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();

            for (int i = 0; i < targets.size();) {
                if (targets.get(i).getFiducialId() == aprilTag_ID) {
                    return i;
                }
            }
        }
        return -1;
    }

    public double distanceFromAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID)).getBestCameraToTarget().getX();
        } else {
            return 1.5;
        }
    }

    public double distanceFromTheMiddleOfTheAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID)).getBestCameraToTarget().getY();
        } else {
            return 0;
        }
    }

    public boolean seesAprilTags() {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            return true;
        } else {
            return false;
        }
    }
}
