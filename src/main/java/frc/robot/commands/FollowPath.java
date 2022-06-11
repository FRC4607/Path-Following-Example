package frc.robot.commands;

import java.nio.file.Path;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FollowPath extends CommandBase {

    private final Trajectory m_trajectory;

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    
    private RamseteCommand command;

    // Reset odometry to the starting pose of the trajectory.

    public FollowPath(DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory) {
        m_trajectory = trajectory;
        m_drivetrainSubsystem = drivetrainSubsystem;
    }

    public FollowPath(DrivetrainSubsystem drivetrain, Path pathweaverJSON) {
        m_drivetrainSubsystem = drivetrain;
        Trajectory trajectory;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(pathweaverJSON);
        } catch (Exception e) {
            System.err.println(e.getMessage());
            trajectory = null;
        }
        m_trajectory = trajectory;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.setBrakeMode(true);
        command = null;
        m_drivetrainSubsystem.resetOdometry(m_trajectory.getInitialPose());
        command = new RamseteCommand(
            m_trajectory,
            m_drivetrainSubsystem::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DrivetrainConstants.ks_Volts,
                DrivetrainConstants.kv_VoltSecondsPerMeters,
                DrivetrainConstants.ka_VoltSecondsSquaredPerMeters),
            DrivetrainConstants.kDriveKinematics,
            m_drivetrainSubsystem::getWheelSpeeds,
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            m_drivetrainSubsystem::tankDriveVolts,
            m_drivetrainSubsystem);
        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_drivetrainSubsystem.tankDriveVolts(0.0, 0.0);
        }
        m_drivetrainSubsystem.setBrakeMode(false);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}