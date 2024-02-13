package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.PIDGains;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private CANSparkMax m_armMotor = new CANSparkMax(Constants.kArmMotor_ID, MotorType.kBrushless);
    private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private SparkPIDController m_armPIDController = m_armMotor.getPIDController();

    private double m_armSetpoint = Constants.kHomePosition;
    private double m_feedforward;

    private TrapezoidProfile m_armProfile = new TrapezoidProfile(Constants.kArmMotionConstraint);
    private TrapezoidProfile.State m_startState = new TrapezoidProfile.State(m_armEncoder.getPosition(), m_armEncoder.getVelocity());
    private TrapezoidProfile.State m_endState = new TrapezoidProfile.State(m_armSetpoint, 0.0);
    private TrapezoidProfile.State m_targetState = new TrapezoidProfile.State(m_armSetpoint, 0.0);
    private Timer m_armTimer = new Timer();

    public Arm() {
        m_armMotor.setInverted(Constants.kArmInverted);
        m_armMotor.setIdleMode(IdleMode.kBrake);
        m_armMotor.setSmartCurrentLimit(Constants.kCurrentLimit);

        m_armEncoder.setPosition(0);
        m_armEncoder.setPositionConversionFactor(Constants.kArmGearRatio);
        m_armEncoder.setVelocityConversionFactor(Constants.kArmVelocityFactor);
        
        m_armPIDController.setOutputRange(-0.1, 0.6);
        PIDGains.setSparkMaxGains(m_armPIDController, Constants.kArmPositionGains);

        m_armTimer.start();
        updateMotionProfile();
    }
    /**
    * Sets the target position and updates the motion profile if the target position changed.
    * @param _setpoint The new target position in radians.
    */
    public void setTargetPosition(double _setpoint){
        if (_setpoint != m_armSetpoint) {
            m_armSetpoint = _setpoint;
            updateMotionProfile();
        }
    }

    private void updateMotionProfile() {
        m_startState = new TrapezoidProfile.State(m_armEncoder.getPosition(), m_armEncoder.getVelocity());
        m_endState = new TrapezoidProfile.State(m_armSetpoint, 0.0);
        m_armProfile = new TrapezoidProfile(Constants.kArmMotionConstraint);
        m_armTimer.reset();  
    }

    public void resetArmEncoder(){
        m_armEncoder.setPosition(0);
    }

    /**
     * Drives the arm to a position using a trapezoidal motion profile.
     * This function is usually wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
     * This function updates the motor position control loop using a setpoint from the trapezoidal motion profile.
     * The target position is the last set position with {@code setTargetPosition}.
     */
    public void runAutomatic() {
        double elapsedTime = m_armTimer.get();
        if (m_armProfile.isFinished(elapsedTime)) {
            m_targetState = new TrapezoidProfile.State(m_armSetpoint, 0.0);
        }
        else {
            m_targetState = m_armProfile.calculate(elapsedTime, m_startState, m_endState);
        }
        m_feedforward = Constants.kArmFeedforward.calculate(
            m_armEncoder.getPosition() + Constants.kArmZeroCosineOffset, m_targetState.velocity);
        m_armPIDController.setReference(
            m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
    }
/* Uncomment for manual arm testing
    public void armMove(double turn){
        SmartDashboard.putNumber("TURN Power (%)", turn);
        
        if(Math.abs(turn) < 0.1){
            turn=0.0;
        }
        turn = Math.copySign(turn * turn, turn);
        m_armMotor.set(turn);
    }
*/

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Current (AMPS)", m_armMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm motor temp (C)", m_armMotor.getMotorTemperature());
    SmartDashboard.putNumber("Arm Pos", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Setpoint", m_armSetpoint);
  }
}