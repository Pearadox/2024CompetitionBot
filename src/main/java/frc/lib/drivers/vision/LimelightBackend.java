// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;

import java.util.Arrays;
import java.util.Optional;

public class LimelightBackend extends VisionBackend {
    private String llName;

    private final DoubleArraySubscriber botPose;
    private final DoubleSubscriber cl;
    private final DoubleSubscriber tl;

    public LimelightBackend(String llName) {
        this.llName = llName;

        botPose =  NetworkTableInstance.getDefault().getTable(llName).getDoubleArrayTopic("botpose_wpiblue").subscribe(null);
        cl = NetworkTableInstance.getDefault().getTable(llName).getDoubleTopic("cl").subscribe(0);
        tl = NetworkTableInstance.getDefault().getTable(llName).getDoubleTopic("tl").subscribe(0);
    }

    @Override
    public Optional<VisionBackend.Measurement> getMeasurement() {
        TimestampedDoubleArray[] updates = botPose.readQueue();

        if (updates.length == 0) {
            return Optional.empty();
        }

        TimestampedDoubleArray update = updates[updates.length - 1];

        if (Arrays.equals(update.value, new double[11])) {
            return Optional.empty();
        }
        
        double x = update.value[0];
        double y = update.value[1];
        double z = update.value[2];
        double roll = Units.degreesToRadians(update.value[3]);
        double pitch = Units.degreesToRadians(update.value[4]);
        double yaw = Units.degreesToRadians(update.value[5]);

        double latency = cl.get() + tl.get();

        double timestamp = (update.timestamp * 1e-6) - (latency * 1e-3);
        Pose3d pose = new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));

        return Optional.of(new Measurement(
            timestamp,
            pose,
            Constants.VisionConstants.LIMELIGHT_STD_DEV
        ));
    }

    public boolean isValid(){
        double[] botpose_wpiblue = NetworkTableInstance.getDefault().getTable(llName).getEntry("botpose_wpiblue").getDoubleArray(new double[11]);

        double tagCount = botpose_wpiblue[7];
        double avgDist = botpose_wpiblue[9];

        return tagCount > 0 && avgDist < 4.5;
    }
}