package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PeriodicCounter;
import frc.robot.Constants.DriveCANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.swerve.RevMaxSwerveModule;

/**
 * Subsystem controlling the swerve drive and it's inputs
 * @author MattaRama
 * @author REVRobotics
*/
public class DriveSubsystem extends SubsystemBase {
    private static boolean driveEnabled = true;

    private PeriodicCounter debugCounter = new PeriodicCounter(10);

    private static final class SwerveModules {
        private static final RevMaxSwerveModule m_frontLeft = new RevMaxSwerveModule(
            DriveCANConstants.kFrontLeftDrivingCanId,
            DriveCANConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);
        private static final RevMaxSwerveModule m_frontRight = new RevMaxSwerveModule(
            DriveCANConstants.kFrontRightDrivingCanId,
            DriveCANConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);
        private static final RevMaxSwerveModule m_backLeft = new RevMaxSwerveModule(
            DriveCANConstants.kRearLeftDrivingCanId,
            DriveCANConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);
        private static final RevMaxSwerveModule m_backRight = new RevMaxSwerveModule(
            DriveCANConstants.kRearRightDrivingCanId,
            DriveCANConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    }

    private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        buildSwerveModulePositions()
    );

    public void setDriveEnabled(boolean value) {
        driveEnabled = value;
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
            Rotation2d.fromDegrees(m_gyro.getAngle()),
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

            SmartDashboard.putNumber("gyro/angle", m_gyro.getAngle());
            SmartDashboard.putNumber("swerve/heading", getHeading());
        });
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            buildSwerveModulePositions(),
            pose
        );
    }

    /**
     * Drive method, taking in joystick positions
     * @return Returns the success value of this operation
     */
    public boolean drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (!driveEnabled) {
            return false;
        }
        
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rot *= DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        SwerveModules.m_frontLeft.setDesiredState(swerveModuleStates[0]);
        SwerveModules.m_frontRight.setDesiredState(swerveModuleStates[1]);
        SwerveModules.m_backLeft.setDesiredState(swerveModuleStates[2]);
        SwerveModules.m_backRight.setDesiredState(swerveModuleStates[3]);

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
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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
        m_gyro.reset();
    }

    /**
     * @return the heading of the robot, in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    /**
     * @return turning rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}