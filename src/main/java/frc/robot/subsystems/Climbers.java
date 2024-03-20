package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Climbers {
    private final CANSparkMax m_climberMotorRight = new CANSparkMax(Constants.kClimberMotorRight_ID, MotorType.kBrushless);
    private final CANSparkMax m_climberMotorLeft = new CANSparkMax(Constants.kClimberMotorLeft_ID, MotorType.kBrushless);

    public Climbers() {
        m_climberMotorRight.setInverted(false);
        m_climberMotorLeft.setInverted(false);

        m_climberMotorRight.setIdleMode(IdleMode.kBrake);
        m_climberMotorLeft.setIdleMode(IdleMode.kBrake);
    }

    public void rightClimberUp(){
        m_climberMotorRight.set(.5);
    }

    public void leftClimberUp(){
        m_climberMotorLeft.set(.5);
    }

    public void rightClimberDown(){
        m_climberMotorRight.set(-0.5);
    }

    public void leftClimberDown(){
        m_climberMotorLeft.set(-0.5);
    }

    public void rightClimberStop(){
        m_climberMotorRight.set(Constants.kMotorOff);
    }

    public void leftClimberStop(){
        m_climberMotorLeft.set(Constants.kMotorOff);
    }
}
