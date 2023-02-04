package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class LEDSubsystem {
    private PWMSparkMax ledController;

    public LEDSubsystem(PWMSparkMax ledController) {
        this.ledController = ledController;
    }

    // value between -1 and 1
    public void setValue(double val) {
        ledController.set(val);
    }
}
