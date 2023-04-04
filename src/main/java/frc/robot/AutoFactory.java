package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoRoutes.AutoRoute3;
import frc.robot.commands.arm.SetArmLockCommand;
import frc.robot.commands.arm.SetArmToPositionCommand;
import frc.robot.commands.auto.AutoLevelSequence;
import frc.robot.commands.claw.RunClawMotorCommand;
import frc.robot.commands.claw.SetClawShutCommand;
import frc.robot.commands.drive.AutoLevelCommand;
import frc.robot.commands.drive.DriveToRampCommand;
import frc.robot.commands.drive.SetWheelLockStateCommand;
import frc.robot.commands.drive.TimedDriveCommand;
import frc.robot.commands.homing.ZeroHeadingCommand;

public class AutoFactory {
    private RobotContainer robotContainer;

    public AutoFactory(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public Command getAutoRoute(String route) {
        switch (route) {
            case "WrapAroundLevel":
                return getAutoRoute1();
            case "Level":
                return getAutoRoute2();
            case "autoRoute3":
                return getAutoRoute3();
            case "ScoreMobilityLevel":
                return getAutoRoute4();
            case "ScoreMobility":
                return getAutoRoute5();
            case "Score":
                return getAutoRoute6();
            default:
                return null;
        }
    }

    // Starts in position 0
    // Wraps around to front of ramp
    // Auto levels
    public Command getAutoRoute1() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(
            "1-1",
            new com.pathplanner.lib.PathConstraints(6, 3)
        );

        Command pathFollowingCommand = robotContainer.driveSubsystem.followTrajectoryCommand(
            trajectory,
            true
        );

        return Commands.sequence(
            pathFollowingCommand,
            new AutoLevelSequence(robotContainer, true, true)/*,
            Commands.runOnce(() -> {
                robotContainer.driveSubsystem.ahrs.setAngleAdjustment(180);
            })*/
        );
    }
    
    // Starts around position 2
    // Auto levels
    public Command getAutoRoute2() {
        return new AutoLevelSequence(robotContainer, true);
    }

    // Starts around position 2, at a pole
    // Drives and places cone
    // Moves to ramp and auto levels
    public Command getAutoRoute3() {
        // get paths
        PathPlannerTrajectory[] trajectories = {
            PathPlanner.loadPath(
                "3-1",
                new com.pathplanner.lib.PathConstraints(4, 2)
            ),
            PathPlanner.loadPath(
                "3-2",
                new com.pathplanner.lib.PathConstraints(4, 2)
            )
        };

        return Commands.sequence(
            new SetArmToPositionCommand(
                robotContainer.clawSubsystem,
                robotContainer.armSubsystem,
                AutoRoute3.ARM_POSITION,
                false
            ),
            robotContainer.driveSubsystem.followTrajectoryCommand(trajectories[0], true),
            new WaitCommand(AutoRoute3.ARM_DROP_WAIT),
            new SetClawShutCommand(robotContainer.clawSubsystem, true),
            new SetArmToPositionCommand(
                robotContainer.clawSubsystem,
                robotContainer.armSubsystem,
                AutoRoute3.ARM_RAISE_POSITION,
                false
            ),
            robotContainer.driveSubsystem.followTrajectoryCommand(trajectories[1], false),
            new SetClawShutCommand(robotContainer.clawSubsystem, false),
            new AutoLevelSequence(robotContainer, true)
        );
    }

    public Command getAutoRoute4() {
        return Commands.sequence(
            // score cube
            /*new SetArmToPositionCommand(
                robotContainer.clawSubsystem,
                robotContainer.armSubsystem,
                0.1,
                false
            ),*/

            new SetClawShutCommand(robotContainer.clawSubsystem, true),
            Commands.runOnce(() -> {
                robotContainer.clawSubsystem.setIntakeMotors(0.3);
            }),
            new WaitCommand(0.5),
            Commands.runOnce(() -> {
                robotContainer.clawSubsystem.setIntakeMotors(0);
            }),
            new SetClawShutCommand(robotContainer.clawSubsystem, false),
            new WaitCommand(0.5),
            
            // exit community
            new TimedDriveCommand(
                robotContainer.driveSubsystem,
                170,
                0.3,
                0,
                0,
                false
            ),
            new WaitCommand(0.25),
            //new SetWheelLockStateCommand(robotContainer, false),

            // auto level
            new AutoLevelSequence(robotContainer, true)
        );
    }
    
    public Command getAutoRoute5() {
        PathPlannerTrajectory rotate180Trajectory = PathPlanner.loadPath(
            "rotate180",
            new com.pathplanner.lib.PathConstraints(1, 0.5)
        );

        return Commands.sequence(
            // score cube
            new SetClawShutCommand(robotContainer.clawSubsystem, true),
            new WaitCommand(0.2),
            Commands.runOnce(() -> {
                robotContainer.clawSubsystem.setIntakeMotors(1);
            }),
            new WaitCommand(0.5),
            Commands.runOnce(() -> {
                robotContainer.clawSubsystem.setIntakeMotors(0);
            }),
            new WaitCommand(0.2),
            new SetClawShutCommand(robotContainer.clawSubsystem, false),
            new WaitCommand(0.5),
            
            // exit community
            new TimedDriveCommand(
                robotContainer.driveSubsystem,
                165,
                0.3,
                0,
                0,
                false
            )/*,

            // rotate and zero heading
            robotContainer.driveSubsystem.followTrajectoryCommand(rotate180Trajectory, false),
            new ZeroHeadingCommand(robotContainer.driveSubsystem)*/
        );
    }

    public Command getAutoRoute6() {
        return Commands.sequence(
            new SetClawShutCommand(robotContainer.clawSubsystem, true),
            new WaitCommand(0.2),
            Commands.runOnce(() -> {
                robotContainer.clawSubsystem.setIntakeMotors(0.2);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {
                robotContainer.clawSubsystem.setIntakeMotors(0);
            }),
            new WaitCommand(1),
            //new SetClawShutCommand(robotContainer.clawSubsystem, false)
            
            new SetArmLockCommand(robotContainer.armSubsystem, robotContainer.clawSubsystem)
        );
    }
}