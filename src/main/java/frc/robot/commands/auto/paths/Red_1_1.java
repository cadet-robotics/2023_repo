package frc.robot.commands.auto.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.AutoLevelSequence;

public class Red_1_1 extends SequentialCommandGroup {
    public Red_1_1(RobotContainer robotContainer) {
        // get drive command
        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(
            "red-1-1",
            new PathConstraints(8, 4)  
        );

        Command pathFollowingCommand = robotContainer.driveSubsystem.followTrajectoryCommand(
            pathTrajectory,
            true
        );

        addCommands(
            pathFollowingCommand,
            new AutoLevelSequence(robotContainer, false)    
        );
    }
}
