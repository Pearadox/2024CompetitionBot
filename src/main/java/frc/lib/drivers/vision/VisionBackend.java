// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Optional;

//From FRC Team 3636 Generals
public abstract class VisionBackend {
    public abstract Optional<Measurement> getMeasurement();

    public static class Measurement {
        public double timestamp;
        public Pose3d pose;
        public Matrix<N3, N1> stdDeviation;

        public Measurement(double timestamp, Pose3d pose, Matrix<N3, N1> stdDeviation) {
            this.timestamp = timestamp;
            this.pose = pose;
            this.stdDeviation = stdDeviation;
        }
    }
}