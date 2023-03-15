package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class SetClawShutCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;
    
    private final boolean shut;

    public SetClawShutCommand(ClawSubsystem clawSubsystem, boolean shut) {
        this.clawSubsystem = clawSubsystem;
        this.shut = shut;
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawShut(shut);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
