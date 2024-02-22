package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_driveLeftLead = new CANSparkMax(Constants.LEFT_LEAD_ID, MotorType.kBrushless);
    private final CANSparkMax m_driveRightLead = new CANSparkMax(Constants.RIGHT_LEAD_ID, MotorType.kBrushless);
    private final CANSparkMax m_driveLeftFollow = new CANSparkMax(Constants.LEFT_FOLLOW_ID, MotorType.kBrushless);
    private final CANSparkMax m_driveRightFollow = new CANSparkMax(Constants.RIGHT_FOLLOW_ID, MotorType.kBrushless);

    private final DifferentialDrive diffDrive = new DifferentialDrive(m_driveLeftLead::set, m_driveRightLead::set);

    private final RelativeEncoder m_leftEncoder = m_driveLeftLead.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_driveRightLead.getEncoder();
    private final RelativeEncoder m_leftEncoderFollow = m_driveLeftFollow.getEncoder();
    private final RelativeEncoder m_rightEncoderFollow = m_driveRightFollow.getEncoder();

    
    private final static Pigeon2 m_gyro = new Pigeon2(Constants.GYRO_ID, Constants.CANBUS_NAME);

    private final Field2d field = new Field2d();

    private final DifferentialDriveOdometry m_odometry; 

    public DriveTrain() {
        initializePigeon2(m_gyro.getConfigurator());
        
        resetEncoders();
        m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

        // Set the second sparks on each side to follow the leader.
        SendableRegistry.addChild(diffDrive, m_driveLeftLead);
        SendableRegistry.addChild(diffDrive, m_driveRightLead);
        m_driveLeftFollow.follow(m_driveLeftLead);
        m_driveRightFollow.follow(m_driveRightLead);

        //Invert one side to ensure wheels are travelling in the same direction
        m_driveLeftLead.setInverted(false);
        m_driveRightLead.setInverted(true);
        
        //Ensures motors brake when idle
        m_driveLeftLead.setIdleMode(IdleMode.kBrake);
        m_driveLeftFollow.setIdleMode(IdleMode.kBrake);
        m_driveRightLead.setIdleMode(IdleMode.kBrake);
        m_driveRightFollow.setIdleMode(IdleMode.kBrake);
    
        m_gyro.getYaw().setUpdateFrequency(100);
        m_gyro.getPitch().setUpdateFrequency(100);
        m_gyro.getRoll().setUpdateFrequency(100);

        m_leftEncoder.getPosition();
        m_rightEncoder.getPosition();
        m_leftEncoderFollow.getPosition();
        m_rightEncoderFollow.getPosition();
        SmartDashboard.putData("Field",field);
    }

    /**
     * Drive the robot using an arcade drive format.
     * @param fwd Forward/Reverse output
     * @param rot Left/Right output
     */
    public void arcadeDrive(double fwd, double rot) {
        SmartDashboard.putNumber("FWD Power (%)", fwd);
        SmartDashboard.putNumber("ROT Power (%)", rot);

        //Deadband - Prevents movement due to controller drift
        if(Math.abs(fwd) < 0.1){
            fwd=0.0;
        }
        if(Math.abs(rot) < 0.1){
            rot=0.0;
        }

        //Squaring the input from the controllers for more control at lower inputs
        fwd = Math.copySign(fwd * fwd, fwd);
        rot = Math.copySign(rot * rot, rot);

        //enables differential drive based on fwd value and rot value
        diffDrive.arcadeDrive(fwd*.75, rot*.75);
    }

    public void resetEncoders(){
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    /**Gets the average distance of the two encoders.
    * @return the average of the two encoder readings
    */
        public double getAverageEncoderDistance() {
        return (getLeftPos() + getRightPos()) / 2.0;
    }

    /**Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     * @param maxOutput the maximum output to which the drive will be constrained
     */
        public void setMaxOutput(double maxOutput) {
        diffDrive.setMaxOutput(maxOutput);
    }

    /*Zeroes the heading of the robot.*/
        public void zeroHeading() {
        m_gyro.reset();
    }

    /*Gyro Information */
    public StatusSignal<Double> getYaw(){
        return m_gyro.getYaw();
    }
    public StatusSignal<Double> getPitch(){
        return m_gyro.getPitch();
    }
    public StatusSignal<Double> getRoll(){
        return m_gyro.getRoll();
    }

    /**Returns the heading of the robot.
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
    }

    /**Returns the turn rate of the robot.
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
    }

    public double getLeftPos(){
        return rotationsToMeters(m_leftEncoder.getPosition());
    }

    public double getRightPos(){
        return rotationsToMeters(m_rightEncoder.getPosition());
    }

    private void initializePigeon2(Pigeon2Configurator cfg) {
        var toApply = new Pigeon2Configuration();
        /*User can change configs if they want, or leave this blank for factory-default*/
        cfg.apply(toApply);
        /* And initialize yaw to 0 */
        cfg.setYaw(0);
    }

    private double rotationsToMeters(double rotations){
        /* Calculate meters traveled per rotation by converting wheel circumference */
        final double metersPerWheelRotation = Units.inchesToMeters(Constants.kWheelCircumference);
        /* Apply gear ratio to input rotations */
        double gearedRotations = rotations / Constants.kDriveGearRatio;
        /* Multiply geared rotations by meters per rotation */
        return gearedRotations * metersPerWheelRotation; 
    }
    /*private double metersToRotations(double meters) {
         //Get circumference of wheel 
        final double circumference = Constants.kWheelDiameterInches * Math.PI;
        // Every rotation of the wheel travels this many inches 
         //So now get the rotations per meter traveled 
        final double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);
         //Now apply wheel rotations to input meters 
        double wheelRotations = wheelRotationsPerMeter * meters;
        // And multiply by gear ratio to get rotor rotations 
        return wheelRotations * Constants.kDriveGearRatio;
      }*/
@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Yaw", getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Gyro Pitch", getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Gyro Roll", getRoll().getValueAsDouble());
    SmartDashboard.putNumber("Left Encoder POS", getLeftPos());
    SmartDashboard.putNumber("Right Encoder POS", getRightPos());

    m_odometry.update(m_gyro.getRotation2d(),
        getLeftPos(),
        getRightPos());
    field.setRobotPose(m_odometry.getPoseMeters());
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
