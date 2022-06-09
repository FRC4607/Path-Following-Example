package frc.robot;

// import java.nio.file.Path;
// import java.util.List;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DrivetrainConstants;

public class Paths {
    // public static final Path testPath = Filesystem.getDeployDirectory().toPath().resolve("./pathplanner/generatedJSON/PathPlanner_Test.wpilib.json");

    /* private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DrivetrainConstants.ks_Volts,
                DrivetrainConstants.kv_VoltSecondsPerMeters,
                DrivetrainConstants.ka_VoltSecondsSquaredPerMeters),
            DrivetrainConstants.kDriveKinematics,
            10); */

    // Create config for trajectory
    /* private static final TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint); */

    // An example trajectory to follow.  All units in meters.
    /* public static final Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config); */
}
