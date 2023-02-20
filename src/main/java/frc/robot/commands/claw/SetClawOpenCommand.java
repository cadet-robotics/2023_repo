package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class SetClawOpenCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;
    
    private final boolean open;

    public SetClawOpenCommand(ClawSubsystem clawSubsystem, boolean open) {
        this.clawSubsystem = clawSubsystem;
        this.open = open;
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawShut(open);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
