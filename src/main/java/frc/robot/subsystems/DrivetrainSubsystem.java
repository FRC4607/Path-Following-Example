package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

/**
 * Drivetrain
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private WPI_TalonFX mFrontLeftMotor;
    private WPI_TalonFX mBackLeftMotor;
    private WPI_TalonFX mFrontRightMotor;
    private WPI_TalonFX mBackRightMotor;

    private MotorControllerGroup mLeftMotorControllerGroup;
    private MotorControllerGroup mRightMotorControllerGroup;
    private DifferentialDrive mDifferentialDrive;

    private WPI_CANCoder mLeftEncoder;
    private WPI_CANCoder mRightEncoder;

    private WPI_PigeonIMU mPigeonIMU;

    private final DifferentialDriveOdometry m_odometry;


    public DrivetrainSubsystem() {
        mFrontLeftMotor = new WPI_TalonFX(DrivetrainConstants.frontLeftMoterID);
        mBackLeftMotor = new WPI_TalonFX(DrivetrainConstants.backLeftMoterID);

        mFrontRightMotor = new WPI_TalonFX(DrivetrainConstants.frontRightMoterID);
        mBackRightMotor = new WPI_TalonFX(DrivetrainConstants.backRightMoterID);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.2);
        motorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        motorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        motorConfig.remoteFilter0.remoteSensorDeviceID = DrivetrainConstants.leftEncoderID;
        motorConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;
        motorConfig.remoteFilter1.remoteSensorDeviceID = DrivetrainConstants.rightEncoderID;

        mFrontLeftMotor.configFactoryDefault();
        mFrontLeftMotor.configAllSettings(motorConfig);
        mFrontLeftMotor.setInverted(false);

        mBackLeftMotor.configFactoryDefault();
        mBackLeftMotor.configAllSettings(motorConfig);
        mBackLeftMotor.setInverted(false);

        mFrontRightMotor.configFactoryDefault();
        mFrontRightMotor.configAllSettings(motorConfig);
        mFrontRightMotor.setInverted(true);

        mBackRightMotor.configFactoryDefault();
        mBackRightMotor.configAllSettings(motorConfig);
        mBackRightMotor.setInverted(true);
        

        mLeftMotorControllerGroup = new MotorControllerGroup(mFrontLeftMotor, mBackLeftMotor);
        mRightMotorControllerGroup = new MotorControllerGroup(mFrontRightMotor, mBackRightMotor);

        mDifferentialDrive = new DifferentialDrive(mLeftMotorControllerGroup, mRightMotorControllerGroup);


        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        encoderConfig.sensorCoefficient = DrivetrainConstants.sensorCoefficient;
        encoderConfig.unitString = "meters";
        
        mLeftEncoder = new WPI_CANCoder(DrivetrainConstants.leftEncoderID);
        mRightEncoder = new WPI_CANCoder(DrivetrainConstants.rightEncoderID);

        mLeftEncoder.configFactoryDefault();
        mLeftEncoder.configAllSettings(encoderConfig);
        mLeftEncoder.configSensorDirection(true);

        mRightEncoder.configFactoryDefault();
        mRightEncoder.configAllSettings(encoderConfig);


        mPigeonIMU = new WPI_PigeonIMU(DrivetrainConstants.pidgeonID);
        mPigeonIMU.setFusedHeading(0);

        m_odometry = new DifferentialDriveOdometry(getRotation2d());
    }

    @Override
    public void periodic() {
        m_odometry.update(getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    /**
     * Returns the current distance the left encoder has traveled in meters.
     * @return The current distance the left encoder has traveled in meters.
     */
    public double getLeftEncoderDistance() {
        return mLeftEncoder.getPosition();
    }

    /**
     * Returns the current distance the right encoder has traveled in meters.
     * @return The current distance the right encoder has traveled in meters.
     */
    public double getRightEncoderDistance() {
        return mRightEncoder.getPosition();
    }

    /**
     * Returns the current speed the left encoder is traveling in meters/second.
     * @return The current speed the left encoder is traveling in meters/second.
     */
    public double getLeftEncoderVelocity() {
        return mLeftEncoder.getVelocity();
    }

    /**
     * Returns the current speed the right encoder is traveling in meters/second.
     * @return The current speed the right encoder is traveling in meters/second.
     */
    public double getRightEncoderVelocity() {
        return mRightEncoder.getVelocity();
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        mDifferentialDrive.arcadeDrive(fwd, rot);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (mLeftEncoder.getPosition() + mRightEncoder.getPosition()) / 2.0;
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        mPigeonIMU.setFusedHeading(0);
    }

    public Rotation2d getRotation2d() {
        // Both angles CCW positive
        return Rotation2d.fromDegrees(mPigeonIMU.getFusedHeading());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getRotation2d());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        mLeftMotorControllerGroup.setVoltage(leftVolts);
        mRightMotorControllerGroup.setVoltage(rightVolts);
        mDifferentialDrive.feed();
    }
    
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    public void setBrakeMode(boolean enabled) {
        if (enabled) {
            mFrontLeftMotor.setNeutralMode(NeutralMode.Brake);
            mFrontRightMotor.setNeutralMode(NeutralMode.Brake);
            mBackLeftMotor.setNeutralMode(NeutralMode.Brake);
            mBackRightMotor.setNeutralMode(NeutralMode.Brake);
        }
        else {
            mFrontLeftMotor.setNeutralMode(NeutralMode.Coast);
            mFrontRightMotor.setNeutralMode(NeutralMode.Coast);
            mBackLeftMotor.setNeutralMode(NeutralMode.Coast);
            mBackRightMotor.setNeutralMode(NeutralMode.Coast);
        }
    }
}