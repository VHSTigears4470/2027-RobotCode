package com.tigears4470.vision;

import java.lang.classfile.ClassFile.Option;
import java.util.Optional;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;

import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateConfig;
import com.limelightvision.PoseEstimateType;

public class LimelightCamera {
    
    private final Limelight limelight;
    
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable ntLimelights = ntInstance.getTable("Limelights");
    private final NetworkTable ntLimelightInstance;
    private final StructPublisher<Pose2d> posePublisher;

    private PoseEstimate[] lastValidMeasurements = new PoseEstimate[]{};
    private boolean initialPoseSet = false;

    public LimelightCamera(String name, Pose3d pose) {
        ntLimelightInstance = ntLimelights.getSubTable(name);
        posePublisher = ntLimelightInstance.getStructTopic("Estimated Pose", Pose2d.struct).publish();
        
        double untrusted = PoseEstimateConfig.UNTRUSTED;
        PoseEstimateConfig mt1Config = PoseEstimateConfig.defaultMT1() // starting with the safe default means you are not required to set every single configuration value
        .withMinTagCount(1)
        .withMaxSingleTagAmbiguity(0.7) // MT1 needs low-ambiguity perspectives
        .withMaxSingleTagDistance(4.0) // If we only see one tag, don't trust it unless we are at most 4m away from it
        .withMaxAvgTagDistance(6.0) // If we see multiple tags, max avg distance must be less than 6 meters.
        .withMinAvgTagArea(0.02) // 0-100, percentage of image area. This is .02%, not 2%.
        .withFieldBounds(16.541, 8.069) // Reject pose estimates that are out of bounds.
        .withFieldBoundsMargin(0.5) // Add .5m margin around field.
        .withStdDevXY(0.5, 0.05, 2.0) // .5 base, absolute min .05, absolute max 2.0. With scaling enabled, .5 at 1m distance
        .withStdDevTheta(untrusted, untrusted, untrusted) // never incorporate pose estimate rotation. You may want to incorporate rotation by setting these to other values
        .withStdDevDistanceScaling(1.0, 0.0, 6.0) // linear scaling, only scale between 0 and 6m.
        .withStdDevTagCountDivision(0.5); // Enhance trust by square root of number of contributing tags
        PoseEstimateConfig mt2Config = PoseEstimateConfig.defaultMT2()
        .withMinTagCount(1)
        .withMaxSingleTagAmbiguity(1.0) // MT2 can handle maximally ambiguous perspectives. Accept tags regardless of ambiguity value.
        .withMaxSingleTagDistance(0.0) // 0 disables this check
        .withMaxAvgTagDistance(8.0)
        .withMinAvgTagArea(0.02) //0-100, percentage of image area
        .withFieldBounds(16.541, 8.069)
        .withFieldBoundsMargin(0.5)
        .withStdDevXY(0.3, 0.0001, 2.0)
        .withStdDevTheta(untrusted, untrusted, untrusted)
        .withStdDevDistanceScaling(0.5, 0.0, 8.0) // Less aggressive STDDev scaling for MT2. Scale by sqrt(distance) rather than distance^1.
        .withStdDevTagCountDivision(0.5); // Enhance trust by a factor equal to the square root of number of contributing tags
        
        limelight = new Limelight(name, pose);
    }

    public PoseEstimate[] getLastValidMeasurements() {
        return lastValidMeasurements;
    }

    public boolean process(int robotYawDegrees) {
        Limelight.setSharedRobotOrientation(robotYawDegrees);

        lastValidMeasurements = new PoseEstimate[]{};
        
        if(RobotState.isDisabled()) {
            limelight.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
            limelight.setFiducialIDFiltersOverride(new int[]{});
            lastValidMeasurements = limelight.readAcceptedPoseEstimates(PoseEstimateType.MT1_WPIBLUE); 
        } else {
            lastValidMeasurements = limelight.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE); 
        }

        if (lastValidMeasurements.length > 0) {
            if(lastValidMeasurements[lastValidMeasurements.length-1].timestampSeconds > 0 &&
               !lastValidMeasurements[lastValidMeasurements.length-1].pose.getTranslation().equals(Translation2d.ZERO)) {
                initialPoseSet = true;
                posePublisher.set(lastValidMeasurements[lastValidMeasurements.length-1].pose);
            }   
        } else if(!initialPoseSet) {
            if(RobotBase.isReal()) {
                DriverStationErrors.reportWarning("[LIMELIGHT]" + limelight.getName() + "not connected", true);
            }
        }

        return lastValidMeasurements.length > 0;
    }
}
