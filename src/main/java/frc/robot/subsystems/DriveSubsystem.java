package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PeriodicCounter;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveCANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.arm.SetArmToPositionCommand;
import frc.robot.swerve.RevMaxSwerveModule;
import frc.robot.utils.SwerveUtil;

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
    public boolean headlessMode = true;

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

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

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
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;
    
        if (rateLimit) {
          // Convert XY to polar for rate limiting
          double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
          double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
    
          // Calculate the direction slew rate based on an estimate of the lateral acceleration
          double directionSlewRate;
          if (m_currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
          } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
          }
          
          double currentTime = WPIUtilJNI.now() * 1e-6;
          double elapsedTime = currentTime - m_prevTime;
          double angleDif = SwerveUtil.AngleDifference(inputTranslationDir, m_currentTranslationDir);
          if (angleDif < 0.45*Math.PI) {
            m_currentTranslationDir = SwerveUtil.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          }
          else if (angleDif > 0.85*Math.PI) {
            if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
              // keep currentTranslationDir unchanged
              m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            else {
              m_currentTranslationDir = SwerveUtil.WrapAngle(m_currentTranslationDir + Math.PI);
              m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }
          }
          else {
            m_currentTranslationDir = SwerveUtil.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
          }
          m_prevTime = currentTime;
          
          xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
          ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
          m_currentRotation = m_rotLimiter.calculate(rot);
    
    
        } else {
          xSpeedCommanded = xSpeed;
          ySpeedCommanded = ySpeed;
          m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * Preferences.getDouble("maxSpeedMetersPerSecond", DriveConstants.kMaxSpeedMetersPerSecond);
        double ySpeedDelivered = ySpeedCommanded * Preferences.getDouble("maxSpeedMetersPerSecond", DriveConstants.kMaxSpeedMetersPerSecond);
        double rotDelivered = m_currentRotation * Preferences.getDouble("maxSpeedMetersPerSecond", DriveConstants.kMaxAngularSpeed);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_ahrs.getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        setModuleStates(swerveModuleStates);
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