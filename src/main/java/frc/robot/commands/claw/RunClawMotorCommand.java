package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class RunClawMotorCommand extends CommandBase {
    private ClawSubsystem clawSubsystem;
    private boolean suck;

    public RunClawMotorCommand(ClawSubsystem clawSubsystem, boolean suck) {
        this.clawSubsystem = clawSubsystem;
        this.suck = suck;
    }

    @Override
    public void initialize() {
        clawSubsystem.setIntakeMotors(ClawConstants.INTAKE_SPEED_MAX * (suck ? -1 : 1));
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setIntakeMotors(0);
    }
}
