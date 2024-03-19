package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);

    private final RelativeEncoder m_leftEncoder = m_driveLeftLead.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_driveRightLead.getEncoder();
    
    private final Pigeon2 m_gyro = new Pigeon2(Constants.GYRO_ID, Constants.CANBUS_NAME);

    private final Field2d field = new Field2d();

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition()); 

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(8.25);
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(1.25);
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    final double GOAL_RANGE_METERS = Units.inchesToMeters(7);
    final PhotonCamera frontCam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    final double LINEAR_P = 2;
    final double LINEAR_D = 0.0;
    
    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;

    double forwardSpeed;
    double rotationSpeed;

    public DriveTrain() {
        initializePigeon2(m_gyro.getConfigurator());

        resetEncoders();

        // Set the second sparks on each side to follow the leader.
        SendableRegistry.addChild(diffDrive, m_driveLeftLead);
        SendableRegistry.addChild(diffDrive, m_driveRightLead);
        m_driveLeftFollow.follow(m_driveLeftLead);
        m_driveRightFollow.follow(m_driveRightLead);

        //Invert one side to ensure wheels are travelling in the same direction
        m_driveLeftLead.setInverted(false);
        m_driveRightLead.setInverted(true);
        
        m_leftEncoder.setPositionConversionFactor(Constants.kEncoderConversionFactor);
        m_leftEncoder.setVelocityConversionFactor(Constants.kEncoderConversionFactor/60);
        m_rightEncoder.setPositionConversionFactor(Constants.kEncoderConversionFactor);
        m_rightEncoder.setVelocityConversionFactor(Constants.kEncoderConversionFactor/60);
                
        //Ensures motors brake when idle
        m_driveLeftLead.setIdleMode(IdleMode.kBrake);
        m_driveRightLead.setIdleMode(IdleMode.kBrake);

        m_driveLeftLead.burnFlash();
        m_driveLeftFollow.burnFlash();
        m_driveRightLead.burnFlash();
        m_driveLeftFollow.burnFlash();
        
        //Align update frequency for gyro
        m_gyro.getYaw().setUpdateFrequency(100);
        m_gyro.getPitch().setUpdateFrequency(100);
        m_gyro.getRoll().setUpdateFrequency(100);
        
// Configure AutoBuilder last
    AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // Current ChassisSpeeds supplier
            this::driveRobotRelative, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this); // Reference to this subsystem to set requirements

        SmartDashboard.putData("Field",field);
    }

    /**Drive the robot using an arcade drive format.
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

    /*Initializes the pigeon and sets it to 0 when starting */
    private void initializePigeon2(Pigeon2Configurator cfg) {
        var toApply = new Pigeon2Configuration();
        /*User can change configs if they want, or leave this blank for factory-default*/
        cfg.apply(toApply);
        /* And initialize yaw to 0 */
        cfg.setYaw(0);
    }

    //Returns the currently-estimated pose of the robot.
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }


    //Return the current wheel speeds of the robot
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
       return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),m_rightEncoder.getVelocity());
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    //Resets the odometry to the specified pose.
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
    }

    //Controls left and right sides of the drive directly with voltages
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_driveLeftLead.setVoltage(leftVolts);
        m_driveRightLead.setVoltage(rightVolts);
        diffDrive.feed();
      }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        arcadeDrive(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().omegaRadiansPerSecond );
    }

    //Set Encoders to read a position of 0
    public void resetEncoders(){
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }
    
    //Gets the average distance of the two encoders.
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    }

    //Returns the left drive encoder
    public RelativeEncoder getLeftEncoder(){
        return m_leftEncoder;
    }
    
    //Returns the left drive encoder
    public RelativeEncoder getRightEncoder(){
        return m_rightEncoder;
    }

    //Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
    public void setMaxOutput(double maxOutput) {
        diffDrive.setMaxOutput(maxOutput);
    }

    //Zeroes the heading of the robot.
    public void zeroHeading() {
        m_gyro.reset();
    }

    //Returns the heading of the robot from -180 to 180
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
        //return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
    }

    //Returns the turn rate of the robot in degrees per second
    public double getTurnRate() {
        return -m_gyro.getRate();
        //return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
    }
    
    public void driveFacing(double power, Rotation2d angle){
        double omega = Constants.headingPID.calculate(
            getPose().getRotation().getRadians(),
            angle.getRadians());
        
        arcadeDrive(power, omega);
    }

@Override
  //This method will be called once per scheduler run
  public void periodic() {
    //Info we want sent to Smartdashboard in periodic block
    SmartDashboard.putNumber("Gyro Yaw", getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Gyro Pitch", getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Gyro Roll", getRoll().getValueAsDouble());
    SmartDashboard.putNumber("Left E.POS", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right E.POS", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left E.SPD", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right E.SPD", m_rightEncoder.getVelocity());

    //Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    //Update robot pose on field
    field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
