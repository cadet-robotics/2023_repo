package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.drive.AutoLevelCommand;
import frc.robot.commands.drive.DriveToRampCommand;
import frc.robot.commands.drive.FinalizeLevelCommand;
import frc.robot.commands.drive.SetWheelLockStateCommand;

public class AutoLevelSequence extends SequentialCommandGroup {
    public AutoLevelSequence(RobotContainer robotContainer, boolean inverse, boolean finalize) {
        addCommands(
            new DriveToRampCommand(robotContainer.driveSubsystem, inverse)/*,
            new AutoLevelCommand(robotContainer.driveSubsystem, inverse),
            new WaitCommand(0.3),
            new SetWheelLockStateCommand(robotContainer, true),
            new WaitCommand(DriveConstants.FINALIZE_START_DELAY)*/
        );

        if (finalize) {
            addCommands(
                new FinalizeLevelCommand(robotContainer.driveSubsystem, inverse)
            );
        }
    }

    public AutoLevelSequence(RobotContainer robotContainer, boolean inverse) {
        this(robotContainer, inverse, true);
    }
}
