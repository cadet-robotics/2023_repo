package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PeriodicCounter;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveCANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.swerve.RevMaxSwerveModule;

/**
 * Subsystem controlling the swerve drive and it's inputs
 * @author MattaRama
 * @author REVRobotics
 * @author Rob Heslin
*/
public class DriveSubsystem extends SubsystemBase {
    private RobotContainer robotContainer;

    private boolean driveEnabled = true;
    public boolean homingMode = false; // TODO: get rid of this

    private PeriodicCounter debugCounter = new PeriodicCounter(10);

    private TeleOpDriveCommand teleOpDriveCommand;

    private final double m_frontLeftOffset;
    private final double m_frontRightOffset;
    private final double m_backLeftOffset;
    private final double m_backRightOffset;

    // TODO: revert to private after testing is done
    public static final class SwerveModules {
        public static final RevMaxSwerveModule m_frontLeft = new RevMaxSwerveModule(
            DriveCANConstants.kFrontLeftDrivingCanId,
            DriveCANConstants.kFrontLeftTurningCanId,
            Preferences.getDouble("swerveFrontLeftOffset", 0.0));
        public static final RevMaxSwerveModule m_frontRight = new RevMaxSwerveModule(
            DriveCANConstants.kFrontRightDrivingCanId,
            DriveCANConstants.kFrontRightTurningCanId,
             Preferences.getDouble("swerveFrontRightOffset", 0.0));
        public static final RevMaxSwerveModule m_backLeft = new RevMaxSwerveModule(
            DriveCANConstants.kRearLeftDrivingCanId,
            DriveCANConstants.kRearLeftTurningCanId,
            Preferences.getDouble("swerveBackLeftOffset", 0.0));
        public static final RevMaxSwerveModule m_backRight = new RevMaxSwerveModule(
            DriveCANConstants.kRearRightDrivingCanId,
            DriveCANConstants.kRearRightTurningCanId,
            Preferences.getDouble("swerveBackRightOffset", 0.0));
    }

    //private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private AHRS m_ahrs = new AHRS();

    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_ahrs.getAngle()),
        buildSwerveModulePositions()
    );

    public DriveSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // pull offset from preferences
        this.m_frontLeftOffset = Preferences.getDouble("swerveFrontLeftOffset", 0.0);
        this.m_frontRightOffset = Preferences.getDouble("swerveFrontRightOffset", 0.0);
        this.m_backLeftOffset = Preferences.getDouble("swerveBackLeftOffset", 0.0);
        this.m_backRightOffset = Preferences.getDouble("swerveBackRightOffset", 0.0);
    }

    public void setDriveEnabled(boolean value) {
        driveEnabled = value;
        if (driveEnabled) {
            teleOpDriveCommand = new TeleOpDriveCommand(this, robotContainer.driverController);
            teleOpDriveCommand.schedule();
        } else {
            if (teleOpDriveCommand != null) {
                teleOpDriveCommand.cancel();
            }
            teleOpDriveCommand = null;
        }
    }

    public boolean getDriveEnabled() {
        return driveEnabled;
    }

    public SwerveModulePosition[] buildSwerveModulePositions() {
        return new SwerveModulePosition[] {
            SwerveModules.m_frontLeft.getPosition(),
            SwerveModules.m_frontRight.getPosition(),
            SwerveModules.m_backLeft.getPosition(),
            SwerveModules.m_backRight.getPosition()
        };
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            SwerveModules.m_frontLeft.getState(),
            SwerveModules.m_frontRight.getState(),
            SwerveModules.m_backLeft.getState(),
            SwerveModules.m_backRight.getState()
        };
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_ahrs.getAngle()),
            buildSwerveModulePositions()
        );

        // debug data
        SwerveModuleState[] states = getSwerveModuleStates();
        
        debugCounter.periodic(() -> {
            SmartDashboard.putNumber("swerve/angles/frontLeft", states[0].angle.getDegrees());
            SmartDashboard.putNumber("swerve/angles/frontRight", states[1].angle.getDegrees());
            SmartDashboard.putNumber("swerve/angles/backLeft", states[2].angle.getDegrees());
            SmartDashboard.putNumber("swerve/angles/backRight", states[3].angle.getDegrees());
    
            SmartDashboard.putNumber("swerve/speed/frontLeft", states[0].speedMetersPerSecond);
            SmartDashboard.putNumber("swerve/speed/frontRight", states[1].speedMetersPerSecond);
            SmartDashboard.putNumber("swerve/speed/backLeft", states[2].speedMetersPerSecond);
            SmartDashboard.putNumber("swerve/speed/backRight", states[3].speedMetersPerSecond);

            // gyro output
            /*SmartDashboard.putNumber("gyro/angle", m_gyro.getAngle());
            SmartDashboard.putNumber("gyro/angleRot2d", m_gyro.getRotation2d().getDegrees());
            SmartDashboard.putNumber("gyro/rate", m_gyro.getRate());
            SmartDashboard.putBoolean("gyro/connected", m_gyro.isConnected());*/

            SmartDashboard.putNumber("swerve/heading", getHeading());

            SmartDashboard.putNumber("swerve/encoders/absolute/frontLeft",
                SwerveModules.m_frontLeft.getTurningAbsoluteEncoder().getPosition());
            SmartDashboard.putNumber("swerve/encoders/absolute/frontRight",
                SwerveModules.m_frontRight.getTurningAbsoluteEncoder().getPosition());
            SmartDashboard.putNumber("swerve/encoders/absolute/backLeft",
                SwerveModules.m_backLeft.getTurningAbsoluteEncoder().getPosition());
            SmartDashboard.putNumber("swerve/encoders/absolute/backRight",
                SwerveModules.m_backRight.getTurningAbsoluteEncoder().getPosition());
            
            SmartDashboard.putBoolean("swerve/driveEnabled", driveEnabled);
            SmartDashboard.putBoolean("swerve/homingMode", homingMode);

            SmartDashboard.putNumber("ahrs/gyro/x", m_ahrs.getRawGyroX());
            SmartDashboard.putNumber("ahrs/gyro/y", m_ahrs.getRawGyroY());
            SmartDashboard.putNumber("ahrs/gyro/z", m_ahrs.getRawGyroZ());
            SmartDashboard.putNumber("ahrs/gyro/angle", m_ahrs.getAngle());
        });
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(m_ahrs.getAngle()),
            buildSwerveModulePositions(),
            pose
        );
    }

    /**
     * Drive method, taking in joystick positions
     * @return Success value of this operation
     */
    public boolean drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (!driveEnabled) {
            return false;
        }
        
        xSpeed *= Preferences.getDouble("maxSpeedMetersPerSecond", DriveConstants.kMaxSpeedMetersPerSecond);
        ySpeed *= Preferences.getDouble("maxSpeedMetersPerSecond", DriveConstants.kMaxSpeedMetersPerSecond);
        rot *= Preferences.getDouble("maxAngularMetersPerSecond", DriveConstants.kMaxAngularSpeed);

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_ahrs.getRotation2d()) // Rotation2d.fromDegrees(m_gyro.getAngle())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

        setModuleStates(swerveModuleStates);

        return true;
    }

    /**
     * Sets the wheels into an X formation in order to prevent movement
     */
    public void setX() {
        SwerveModules.m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        SwerveModules.m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        SwerveModules.m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        SwerveModules.m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, Preferences.getDouble("maxSpeedMetersPerSecond", DriveConstants.kMaxSpeedMetersPerSecond));
        SwerveModules.m_frontLeft.setDesiredState(desiredStates[0]);
        SwerveModules.m_frontRight.setDesiredState(desiredStates[1]);
        SwerveModules.m_backLeft.setDesiredState(desiredStates[2]);
        SwerveModules.m_backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to read a position of zero
     */
    public void resetEncoders() {
        SwerveModules.m_frontLeft.resetEncoders();
        SwerveModules.m_backLeft.resetEncoders();
        SwerveModules.m_frontRight.resetEncoders();
        SwerveModules.m_backRight.resetEncoders();
    }

    /**
     * Zeros the heading of the robot
     */
    public void zeroHeading() {
        m_ahrs.reset();
    }

    /**
     * @return the heading of the robot, in degrees, from -180 to 180
     */
    public double getHeading() {
        //return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
        return m_ahrs.getRotation2d().getDegrees();
    }

    /**
     * @return turning rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Sets the zero positions for all the motors
     */
    public void setModuleZeros() {
       
        // get the current angle and add to the offset, the result is a new offset, robot must be restarted for module to accept
        double frontLeftOffset = SwerveModules.m_frontLeft.getState().angle.getRadians() + this.m_frontLeftOffset;
        // set the new offset into Prefferences, to be recalled next boot
        Preferences.setDouble("swerveFrontLeftOffset", frontLeftOffset);

        double frontRightOffset = SwerveModules.m_frontRight.getState().angle.getRadians() + this.m_frontRightOffset;
        Preferences.setDouble("swerveFrontRightOffset", frontRightOffset);

        double backRightOffset = SwerveModules.m_backRight.getState().angle.getRadians() + this.m_backRightOffset;
        Preferences.setDouble("swerveBackRightOffset", backRightOffset);

        double backLeftOffset = SwerveModules.m_backLeft.getState().angle.getRadians() + this.m_backLeftOffset;
        Preferences.setDouble("swerveBackLeftOffset", backLeftOffset);
    }
}