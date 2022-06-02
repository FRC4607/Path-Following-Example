package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcradeDrive extends CommandBase {
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private XboxController mDriver;

    public ArcradeDrive(DrivetrainSubsystem drivetrainSubsystem, XboxController driver) {
        mDrivetrainSubsystem = drivetrainSubsystem;
        mDriver = driver;
    }

    @Override
    public void execute() {
        mDrivetrainSubsystem.arcadeDrive(mDriver.getLeftY(), mDriver.getLeftX());
    }
}
