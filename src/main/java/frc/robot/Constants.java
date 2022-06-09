// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants {
        public static final int frontLeftMoterID = 0;
        public static final int backLeftMoterID = 1;
        public static final int frontRightMoterID = 3;
        public static final int backRightMoterID = 4;
        
        /* public static final int leftEncoderID = 2;
        public static final int rightEncoderID = 5;
        public static final double sensorCoefficient = (Math.PI * Units.inchesToMeters(6)) / 4096;

        public static final int pidgeonID = 6; */

        /* public static final double ks_Volts = 0.65288;
        public static final double kv_VoltSecondsPerMeters = 2.5016;
        public static final double ka_VoltSecondsSquaredPerMeters = 0.38331;
        public static final double trackWidth_Meters = 0.7036911491;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            trackWidth_Meters); */

        // public static final double kPDriveVel = 2.9025 * 2;
    }

    /* public static final class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = .5;
    } */
}
