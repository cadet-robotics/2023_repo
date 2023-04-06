package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDegreesCommand extends CommandBase{
    private RobotContainer robotContainer;
    private double degreeDelta, speed;

    private double startHeading;

    // speed MUST be from 0-1
    public RotateDegreesCommand(RobotContainer robotContainer, double degreeDelta, double speed) {
        this.robotContainer = robotContainer;
        this.degreeDelta = degreeDelta;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        robotContainer.driveSubsystem.setDriveEnabled(false);
        startHeading = robotContainer.driveSubsystem.getHeading() + 180;

        int negate = degreeDelta > 0 ? 1 : -1;
        robotContainer.driveSubsystem.drive(
            0,
            0,
            speed * negate,
            true,
            false,
            true
        );
    }

    @Override
    public boolean isFinished() {
        double heading = robotContainer.driveSubsystem.getHeading() + 180;
        System.out.println("Goal: " + (degreeDelta + startHeading) + " | Current: " + heading);
        if (degreeDelta > 0) {
            return degreeDelta + startHeading <= heading;
        } else {
            return degreeDelta + startHeading >= heading;
        }
    }
}
