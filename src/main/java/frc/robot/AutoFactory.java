package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoRoutes.AutoRoute3;
import frc.robot.commands.arm.SetArmToPositionCommand;
import frc.robot.commands.auto.AutoLevelSequence;
import frc.robot.commands.claw.SetClawOpenCommand;

public class AutoFactory {
    private RobotContainer robotContainer;

    public AutoFactory(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public Command getAutoRoute(String route) {
        switch (route) {
            case "autoRoute1":
                return getAutoRoute1();
            case "autoRoute2":
                return getAutoRoute2();
            case "autoRoute3":
                return getAutoRoute3();
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
            new com.pathplanner.lib.PathConstraints(8, 4)
        );

        Command pathFollowingCommand = robotContainer.driveSubsystem.followTrajectoryCommand(
            trajectory,
            true
        );

        return Commands.sequence(
            pathFollowingCommand,
            new AutoLevelSequence(robotContainer, false)
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
            new SetClawOpenCommand(robotContainer.clawSubsystem, true),
            new SetArmToPositionCommand(
                robotContainer.clawSubsystem,
                robotContainer.armSubsystem,
                AutoRoute3.ARM_RAISE_POSITION,
                false
            ),
            robotContainer.driveSubsystem.followTrajectoryCommand(trajectories[1], false),
            new SetClawOpenCommand(robotContainer.clawSubsystem, false),
            new AutoLevelSequence(robotContainer, true)
        );
    }
}