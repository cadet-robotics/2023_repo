package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.AutoLevelCommand;
import frc.robot.commands.drive.DriveToRampCommand;
import frc.robot.commands.drive.FinalizeLevelCommand;

public class AutoLevelSequence extends SequentialCommandGroup {
    public AutoLevelSequence(RobotContainer robotContainer, boolean inverse) {
        addCommands(
            new DriveToRampCommand(robotContainer.driveSubsystem, inverse),
            new AutoLevelCommand(robotContainer.driveSubsystem, inverse),
            new WaitCommand(DriveConstants.FINALIZE_START_DELAY),
            new FinalizeLevelCommand(robotContainer.driveSubsystem, inverse)
        );
    }
}
