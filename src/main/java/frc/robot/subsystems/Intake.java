package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.kIntakeMotor_ID, MotorType.kBrushless);
    private final DigitalInput m_rightLimitSwitch = new DigitalInput(0);
    
    public Intake() {
        //Setting default settings for motor
        m_IntakeMotor.setInverted(true);
        m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    //Turns motor on to intake based on constant value
    public void IntakeFeed() {
        m_IntakeMotor.set(Constants.kIntakeInPOW);
    }

    //Turns intake motor off
    public void IntakeStop() {
        m_IntakeMotor.set(Constants.kMotorOff);
    }
    public void IntakeShoot() {
        m_IntakeMotor.set(Constants.kIntakeOutPOW);
    }

    public boolean isNoteIn(){
        return m_rightLimitSwitch.get() == true;
    }
}



