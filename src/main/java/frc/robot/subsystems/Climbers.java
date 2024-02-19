package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Climbers {
    private final CANSparkMax m_climberMotorRight = new CANSparkMax(Constants.kClimberMotorRight_ID, MotorType.kBrushless);
    private final CANSparkMax m_climberMotorLeft = new CANSparkMax(Constants.kClimberMotorLeft_ID, MotorType.kBrushless);

    private final RelativeEncoder m_climberRightEncoder = m_climberMotorRight.getEncoder();
    private final RelativeEncoder m_climberLeftEncoder = m_climberMotorLeft.getEncoder();

public Climbers() {
    resetEncoders();

    m_climberMotorRight.setInverted(false);
    m_climberMotorLeft.setInverted(false);

    m_climberMotorRight.setIdleMode(IdleMode.kBrake);
    m_climberMotorLeft.setIdleMode(IdleMode.kBrake);

    m_climberRightEncoder.getPosition();
    m_climberLeftEncoder.getPosition();
}

public void resetEncoders(){
    m_climberRightEncoder.setPosition(0);
    m_climberLeftEncoder.setPosition(0);
}
public void climberUp(){
    m_climberMotorLeft.set(.1);
    m_climberMotorRight.set(.1);
}
public void climberDown(){
    m_climberMotorLeft.set(-.1);
    m_climberMotorRight.set(-.1);
}
public void climberStop(){
    m_climberMotorLeft.set(0);
    m_climberMotorRight.set(0);
}
}
