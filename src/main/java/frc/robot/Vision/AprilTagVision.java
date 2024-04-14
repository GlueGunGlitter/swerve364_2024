// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class AprilTagVision {

    // create photonvision camera 
    PhotonCamera aprilTagsCamera;

    public AprilTagVision() {
        
        // set the photonvision camera
        aprilTagsCamera = new PhotonCamera("AprilTags-Camera");
    }

    /* 
     get the wanted aprilTag on locion on the photonvisin dashbord
     if it dose not see aperilTag it will retern -1 
    */
    public int seesSpecificAprilTag(int aprilTag_ID, PhotonPipelineResult result) {
        if (result.hasTargets()) {
            var targets = result.getTargets();

            for (int i = 0; i < targets.size(); i++) {
                if (targets.get(i).getFiducialId() == aprilTag_ID) {
                    return i;
                }
            }
        }
        return -1;
    }

    /*
     get the forward distance from the camera to whanted aprilTag 
     if the camera dose not see aprilTag it will return 10 
    */
    public double distanceFromAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID, result)).getBestCameraToTarget().getX();
        } else {
            return 10;
        }
    }

    /*
     get the horizontal destance from the cameera to wanted aprilTag 
     if it dose not see aprilTags it will return 0
    */
    public double distanceFromTheMiddleOfTheAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID, result)).getBestCameraToTarget().getY();
        } else {
            return 0;
        }
    }

    // chack if the camera see aprilTag
    public boolean seesAprilTags() {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            return true;
        } else {
            return false;
        }
    }
}
