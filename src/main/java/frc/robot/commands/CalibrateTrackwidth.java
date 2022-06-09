/* package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CalibrateTrackwidth extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_startingRotation;
    private double m_endingRoatition;

    public CalibrateTrackwidth(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.resetEncoders();
        m_startingRotation = m_drivetrainSubsystem.getRotation2d().getDegrees();
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.tankDriveVolts(-2, 2);
    }

    @Override
    public void end(boolean interrupted) {
        m_endingRoatition = m_drivetrainSubsystem.getRotation2d().getDegrees();

        double AverageDistanceTravled = (m_drivetrainSubsystem.getLeftEncoderDistance() - m_drivetrainSubsystem.getRightEncoderDistance()) / 2;
        double rotations = (m_endingRoatition - m_startingRotation)/360;
        double trackWidth = -AverageDistanceTravled / (rotations * Math.PI);

        SmartDashboard.putNumber("Track Width", trackWidth);
        // SmartDashboard.putNumber("Number of Rotations", rotations);
        // SmartDashboard.putNumber("Average Distave Travled", AverageDistanceTravled);
    }

} */
