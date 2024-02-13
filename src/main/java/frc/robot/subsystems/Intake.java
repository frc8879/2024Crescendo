package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_IntakeMotor = new CANSparkMax(Constants.kIntakeMotor_ID, MotorType.kBrushless);
    
    private final RelativeEncoder m_IntakeEncoder = m_IntakeMotor.getEncoder();

public Intake() {
    resetEncoders();
    
    m_IntakeMotor.setInverted(true);
    
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    
    m_IntakeEncoder.getPosition();
    }
public void IntakeFeed() {
    m_IntakeMotor.set(-0.5);
}
public void IntakeStop() {
    m_IntakeMotor.set(0);
}
public void IntakeShoot() {
    m_IntakeMotor.set(0.5);
}

public void resetEncoders(){
    m_IntakeEncoder.setPosition(0);

}


    //SmartDashboard.putNumber("Intake pow(%)", percent);
    //SmartDashboard.putNumber("Intake Motor Current (amps)", IntakeMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Intake Motor Temperature (C)", IntakeMotor.getMotorTemperature());

public double getAmps() {
    return m_IntakeMotor.getOutputCurrent();
}
}

