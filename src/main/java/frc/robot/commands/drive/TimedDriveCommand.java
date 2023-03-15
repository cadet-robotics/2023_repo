package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    private int ticks;
    private double x, y, z;
    private boolean fieldRelative;

    private int ticksElapsed;

    public TimedDriveCommand(DriveSubsystem driveSubsystem, int ticks, double x, double y, double z, boolean fieldRelative) {
        this.driveSubsystem = driveSubsystem;

        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.z = z;
        this.fieldRelative = fieldRelative;

        ticksElapsed = 0;
    }

    @Override
    public void initialize() {
        driveSubsystem.setDriveEnabled(false);

        driveSubsystem.drive(x, y, z, fieldRelative, false, true);
    }

    @Override
    public void execute() {
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return ticksElapsed >= ticks;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, fieldRelative, false, true);
    }
}
