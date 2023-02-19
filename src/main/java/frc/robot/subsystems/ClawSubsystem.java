package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PneumaticsConstants;

/**
 * Controls the grabber 
 * 
 * @author MattaRama
 */
public class ClawSubsystem extends SubsystemBase {
    private RobotContainer robotContainer;

    private CANSparkMax intakeLeft,
                        intakeRight;

    private DoubleSolenoid leftSolenoid,
                           rightSolenoid;

    public ClawSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

        this.intakeLeft = new CANSparkMax(CANConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushed);
        this.intakeRight = new CANSparkMax(CANConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushed);

        // TODO: check for pneumatics hub CAN ID
        this.leftSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            PneumaticsConstants.LEFT_SOLENOID_FORWARD,
            PneumaticsConstants.LEFT_SOLENOID_REVERSE
        );

        this.rightSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            PneumaticsConstants.RIGHT_SOLENOID_FORWARD,
            PneumaticsConstants.RIGHT_SOLENOID_REVERSE
        );
    }
}
