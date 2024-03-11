package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class Shooter {
    private final CANSparkMax m_ShooterMotor = new CANSparkMax(Constants.kRightShooterMotor_ID, MotorType.kBrushless);
    private final CANSparkMax m_ShooterMotor2 = new CANSparkMax(Constants.kLeftShooterMotor_ID, MotorType.kBrushless);
    private final RelativeEncoder m_ShooterEncoder = m_ShooterMotor.getEncoder();
    private final RelativeEncoder m_ShooterEncoder2 = m_ShooterMotor2.getEncoder();
public Shooter() {
    resetEncoders();
    
    m_ShooterMotor.setInverted(true);
    m_ShooterMotor2.setInverted(false);
    m_ShooterMotor.setIdleMode(IdleMode.kCoast);
    m_ShooterMotor2.setIdleMode(IdleMode.kCoast);
    m_ShooterEncoder.getPosition();
    m_ShooterEncoder2.getPosition();
}

public void resetEncoders(){
    m_ShooterEncoder.setPosition(0);
    m_ShooterEncoder2.setPosition(0);
}
public void activeShooter(){
    m_ShooterMotor.set(0.75);
    m_ShooterMotor2.set(0.75);
}
public void stopShooter(){
    m_ShooterMotor.set(0);
    m_ShooterMotor2.set(0);
}
}








