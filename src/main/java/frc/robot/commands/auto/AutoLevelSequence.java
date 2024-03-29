package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoLevelCommand;
import frc.robot.commands.drive.AutoRateLevelCommand;
import frc.robot.commands.drive.DriveToRampCommand;
import frc.robot.commands.drive.DriveUntilPitchCommand;

public class AutoLevelSequence extends SequentialCommandGroup {
    public AutoLevelSequence(RobotContainer robotContainer, boolean inverse, boolean finalize) {
        addCommands(
            new DriveToRampCommand(robotContainer.driveSubsystem, inverse),
            new WaitCommand(1.25),
            new AutoRateLevelCommand(robotContainer, inverse),
            new WaitCommand(1.25),
            new AutoLevelCommand(robotContainer.driveSubsystem, inverse)/*,
            new SetWheelLockStateCommand(robotContainer, true),
            new WaitCommand(DriveConstants.FINALIZE_START_DELAY)*/
        );

        if (finalize) {
            addCommands(
                new WaitCommand(1.5),
                new DriveUntilPitchCommand(robotContainer, 0, 0.11)
            );
        }
    }

    public AutoLevelSequence(RobotContainer robotContainer, boolean inverse) {
        this(robotContainer, inverse, true);
    }
}
