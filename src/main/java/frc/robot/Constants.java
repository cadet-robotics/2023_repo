// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.drive.AutoRateLevelCommand;

// TODO: fix casing 
public final class Constants
{
    public static final class IOConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int CODRIVER_CONTROLLER_PORT = 1;
        public static final int HOMING_CONTROLLER_PORT = 2;

        // driver controller
        // TODO: move zero heading control from homing to driver/codriver
        public static final class DriverControllerConsts {
            public static final double DEADZONE = 0.065;

            public static final int LEFT_JOYSTICK_X = 0;
            public static final int LEFT_JOYSTICK_Y = 1;
            public static final int RIGHT_JOYSTICK_X = 2;

            // TODO: remap this to digital signal
            public static final int FINE_CONTROL_AXIS = 4; // Right trigger
            public static final double FINE_CONTOL_THRESHOLD = 0.2;

            // TODO: remap this to digital signal
            public static final int HEADED_MODE_AXIS = 3;
            public static final double HEADED_MODE_THRESHOLD = 0.2;

            public static final int ZERO_HEADING_BUTTON = 4; // Triangle

            public static final int AUTO_LEVEL_BUTTON = 1; // Square

            //public static final int SET_HEADED_MODE = 7;
            //public static final int SET_HEADLESS_MODE = 8;

            public static final int BEGIN_AUTO_LEVEL_1 = 5;
            public static final int BEGIN_AUTO_LEVEL_2 = 6;
        }

        // codriver controller
        public static final class CoDriverControllerConsts {
            public static final int CLAW_CLOSE_BUTTON = 5; // L1
            public static final int CLAW_OPEN_BUTTON = 6; // R1

            public static final int WHEEL_LOCK_BUTTON = 13; // ps4 logo button
            public static final int WHEEL_UNLOCK_BUTTON = 14; // touchpad button in

            public static final int ARM_LOCK = 4; // Triangle

            
            public static final int LED_MODIFIER_AXIS = 4;
            public static final double LED_MODIFIER_THRESHOLD = 0.2;
            public static final int VIOLET_LIGHT = 1; // Square
            public static final int RED_LIGHT = 3; // Circle
            public static final int BLUE_LIGHT = 2; // Cross
            public static final int YELLOW_LIGHT = 4; // Triangle

            public static final int ARM_MANUAL_AXIS = 5; // right Y axis
            public static final double ARM_MANUAL_DEADZONE = 0.10;

            public static final int CLAW_MANUAL_AXIS = 1; // left Y axis
            public static final double CLAW_MANUAL_DEADZONE = 0.10;
        }

        // homing controller
        public static final class HomingControllerConsts {
            public static final int ROTATE_BL_BUTTON = 1; // X
            public static final int ROTATE_BR_BUTTON = 2; // A
            public static final int ROTATE_FR_BUTTON = 3; // B
            public static final int ROTATE_FL_BUTTON = 4; // Y

            public static final int ZERO_HEADING_BUTTON = 5; // LB
            public static final int RESET_ENCODERS_BUTTON = 6; // RB

            public static final int COARSE_HOMING_SPEED = 7; // LT

            public static final int ZERO_SWERVE_MODULES = 8; // RT      
            
            public static final int EXIT_HOMING_MODE = 9; // back
            public static final int INSERT_HOMING_MODE = 10; // start
        }
    }

    public static final class DriveCANConstants {
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 13;
        public static final int kFrontRightDrivingCanId = 15;
        public static final int kRearRightDrivingCanId = 17;

        public static final int kFrontLeftTurningCanId = 10;
        public static final int kRearLeftTurningCanId = 12; 
        public static final int kFrontRightTurningCanId = 14;
        public static final int kRearRightTurningCanId = 16;
    }

    public static final class PWMConstants {
        public static final int LED_PWM = 0;
    }

    public static final class AnalogConstants {
        public static final int DISTANCE_SENSOR = 0;
    }
    
    public static final class CANConstants {
        public static final int ARM_MAIN_MOTOR = 30;
        public static final int ARM_HELPER_MOTOR = 31;

        public static final int INTAKE_MOTOR_LEFT = 32;
        public static final int INTAKE_MOTOR_RIGHT = 33;
    }

    public static final class PneumaticsConstants {
        public static final int LEFT_SOLENOID_FORWARD = 5; 
        public static final int LEFT_SOLENOID_REVERSE = 3; 
        public static final int RIGHT_SOLENOID_FORWARD = 6; 
        public static final int RIGHT_SOLENOID_REVERSE = 4;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 5; // 4.8
        public static final double kMaxAngularSpeed = 1.75 * Math.PI; // radians per second // 1.75

        public static final double kDirectionSlewRate = 2.5; // radians per second  // original 1.2
        public static final double kMagnitudeSlewRate = 6.0; // percent per second (1 = 100%) // 1.8
        public static final double kRotationalSlewRate = 5.0; // percent per second (1 = 100%) // 2.0

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(28.5); // what is this number
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Default encoder offsets, in case preferences fails
        public static final double frontLeftOffset = 0.0;//1.68753;
        public static final double frontRightOffset = 0.0;//.279077;
        public static final double backLeftOffset = 0;//3.17916;
        public static final double backRightOffset = 0;

        public static final boolean kGyroReversed = false;

        public static final double FINE_SPEED_REDUCTION = 0.3;

        public static final float GYRO_PITCH = -42.4f;

        
        // new autolevel constants
        public static final double AUTO_LEVEL_DEFAULT_SPEED = 0.09;
        public static final Float AUTO_LEVEL_END_RATE 
            = AutoRateLevelCommand.toDegreesPerTick(4.5f); // unit: Δdegrees/s

        // old autolevel stuff mostly, probably should delete at some point to reduce confusion
        public static final double AUTO_LEVEL_COARSE_SPEED = 0.225;
        public static final double AUTO_LEVEL_ANGULAR_MARGIN = 2.5;

        public static final double AUTO_LEVEL_APPROACH_SPEED = 0.3;

        public static final double AUTO_LEVEL_ENTRY_ANGLE = 14;

        public static final double AUTO_LEVEL_FINE_ANGLE = 5; // 7.5
        public static final double AUTO_LEVEL_FINE_SPEED = 0.125;

        public static final double AUTO_LEVEL_GOAL_PITCH = -1;

        public static final int AUTO_LEVEL_TICK_DELAY = 7;
        
        public static final float DRIVE_UNTIL_PITCH_MARGIN = 1.5f;

        // old, should be removed
        public static final double FINALIZE_LEVEL_SPEED = 0.21;
        public static final int FINALIZE_LEVEL_TIME = 30; // 10, 15, 25
        public static final double FINALIZE_START_DELAY = 1.75; // .75
    } 
    
    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    
        public static final double kDrivingP = 0.04; // originally 0.04
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1.0 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 0.5;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;

        public static final double HOMING_SPEED_COARSE = 0.15;
        public static final double HOMING_SPEED_FINE = 0.025;

        public static final double ARM_SPEED = 0.1;
    }

    public static final class LEDColors {
        public static final double RED = 0.61;
        public static final double VIOLET = 0.91;
        public static final double BLUE = 0.85;
        public static final double YELLOW = 0.69;
    }

    public static final class ArmConstants {
        // TODO: find more accurate values for these
        public static final double MAX_POSITION = 0.3901;
        public static final double MIN_POSITION = 0.015573590993881226;
        public static final double RANGE = MAX_POSITION - MIN_POSITION;
        
        // relative position of the arm when it is fully vertical
        public static final double FULL_DOWN_POSITION = 0.1036;

        // The downward position where the intake has to close in order to retract the arm all the way in
        // This position is RELATIVE
        public static final double INTAKE_RETRACT_POSITION = 0.07;
        
        public static final int WAIT_TICKS = 25;

        public static final double RELATIVE_ACCEPTABLE_MARGIN = 0.05;
        public static final double FINE_SPEED_MARGIN = 0.1;

        public static final double COARSE_SPEED_UP = 0.35;
        public static final double COARSE_SPEED_DOWN = 0.2;
        public static final double FINE_SPEED_UP = 0.08;
        public static final double FINE_SPEED_DOWN = 0.00;

        public static final double REVERSE_SPEED_ADDITION = 0.01;

        public static final double LOCKING_SPEED = 0.275;
        public static final double FINE_LOCKING_SPEED = 0.25;
        public static final double LOCKING_MARGIN = 0.0005;

        public static final double MANUAL_ARM_SPEED_MAX = 0.4;

        // arm preset positions
        public static final double[] ARM_PRESET_POSITIONS = {
            0.35,  // .2
            0.81, // .78
            0.98 // 
        };
    }

    public static final class ClawConstants {
        public static final double INTAKE_SPEED_MAX = 0.40; // TODO: check the tuning of this value
    }

    public static final class AutoRoutes {
        // Adds options for the autonomous SendableChooser
        public static final String[] AUTO_ROUTES = {
            "WrapAroundLevel",
            "Level",
            "ScoreMobilityLevel",
            "ScoreMobility",
            "Score",
            "HScore"
        };

        public static final class AutoRoute3 {
            public static final double ARM_POSITION = 0.90;
            public static final double ARM_RAISE_POSITION = 0.95;
            public static final double ARM_DROP_WAIT = 4;
        }
    }
}
