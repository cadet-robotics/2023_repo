package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PeriodicCounter;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveCANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.TeleOpDriveCommand;
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

    // ahrs Gyro Constants
    private final int ANGLE_CHECK_DELAY = 100;
    private final int OFFSET_ANGLE = 45;

    private boolean driveEnabled = true;
    public boolean homingMode = false; // TODO: get rid of this
    //public boolean headlessMode = true;

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
            Preferences.getDouble("swerveFrontLeftOffset", DriveConstants.frontLeftOffset));
        public static final RevMaxSwerveModule m_frontRight = new RevMaxSwerveModule(
            DriveCANConstants.kFrontRightDrivingCanId,
            DriveCANConstants.kFrontRightTurningCanId,
            Preferences.getDouble("swerveFrontRightOffset", DriveConstants.frontRightOffset));
        public static final RevMaxSwerveModule m_backLeft = new RevMaxSwerveModule(
            DriveCANConstants.kRearLeftDrivingCanId,
            DriveCANConstants.kRearLeftTurningCanId,
            Preferences.getDouble("swerveBackLeftOffset", DriveConstants.backLeftOffset));
        public static final RevMaxSwerveModule m_backRight = new RevMaxSwerveModule(
            DriveCANConstants.kRearRightDrivingCanId,
            DriveCANConstants.kRearRightTurningCanId,
            Preferences.getDouble("swerveBackRightOffset", DriveConstants.backRightOffset));
    }

    //private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    public AHRS ahrs = new AHRS();

    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        ahrs.getRotation2d(),
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
        // Offset the Gyro to equal 0 when tilted on the bot
        //ahrs.setAngleAdjustment(OFFSET_ANGLE);
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
            ahrs.getRotation2d(),
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

            SmartDashboard.putNumber("ahrs/gyro/x", ahrs.getRawGyroX());
            SmartDashboard.putNumber("ahrs/gyro/y", ahrs.getRawGyroY());
            SmartDashboard.putNumber("ahrs/gyro/z", ahrs.getRawGyroZ());
            SmartDashboard.putNumber("ahrs/gyro/angle", ahrs.getAngle());
            SmartDashboard.putNumber("ahrs/pitch", ahrs.getPitch());
        });
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            ahrs.getRotation2d(),
            buildSwerveModulePositions(),
            pose
        );
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit, false);
    }

    /**
     * Drive method, taking in joystick positions
     * @return Success value of this operation
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit, boolean override) {
        if (!driveEnabled && !override) {
            return;
        }

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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, ahrs.getRotation2d())
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
        ahrs.reset();
    }

    /**
     * @return the heading of the robot, in degrees, from -180 to 180
     */
    public double getHeading() {
        //return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
        return ahrs.getRotation2d().getDegrees();
    }

    /**
     * @return turning rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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

    // TODO: move auto route to correct side, remove team-dependant paths
    public Command followTrajectoryCommand(PathPlannerTrajectory path, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (isFirstPath) {
                    this.resetOdometry(path.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                path,
                this::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                this::setModuleStates,
                true
            )
        );
    }

    /**
     * Checks the current pitch every 100ms as we are moving towards
     * the desired pitch
     *
     * Note: makes the assumption that we are beginning with a pitch that is greater than the desired
     *
     * @param desiredPitch The angle that we want the robot to reach to change motion
     */
    /*public void waitForPitch(double desiredPitch)
    {
        while(ahrs.getPitch() + OFFSET_ANGLE > desiredPitch)
        {
             try
             {
                 Thread.sleep(ANGLE_CHECK_DELAY);
             }
             catch (InterruptedException e) {
                 // don't expect, ignore handling
             }
        }
    }*/

    /**
     * Go up the ramp at a slow speed, if angle exceeds a number start going backwards to
     * correct it but if the angle is within a level range then stop all motors
     */
    /*public void autoLeveling()
    {
        // disable teleop drive command
        setDriveEnabled(false);

        //Move forward until angle changes
        drive(0, -0.1, 0, true, false);
        // TODO: test to see if its the correct angle
        waitForPitch(50);

        // Slow the speed by 1/4 when the angle changes as the bot starts going up the ramp
        drive(0, -0.025, 0, true, false);

        //if the bot becomes level stop moving the motors

        waitForPitch(0);
        drive(0, 0, 0, true, false);

        // lock wheels
        setX();
    }*/
}
