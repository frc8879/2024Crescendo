// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /*General Constants */
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double kMotorOff = 0.0;
    public static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /*CAN BUS IDs */
    public static String CANBUS_NAME = "rio";
    
    public static final int LEFT_LEAD_ID = 3;
    public static final int RIGHT_LEAD_ID = 1;
    public static final int LEFT_FOLLOW_ID = 4;
    public static final int RIGHT_FOLLOW_ID = 2;

    public static final int GYRO_ID = 5;

    public static final int kArmMotor_ID = 6;
    public static final int kIntakeMotor_ID = 7;
    
    public static final int kRightShooterMotor_ID = 8;
    public static final int kLeftShooterMotor_ID = 9;

    public static final int kClimberMotorRight_ID = 10;
    public static final int kClimberMotorLeft_ID = 11;

  /*Drivetrain Constants */
    public static final double kDriveGearRatio = 8.46; //2 REV-21-1650s connected to single output
    public static final int    kDriveEncoderCPR = 42; //used Hall-Sensor Encoder Resolution from data sheet for REV-21-1650
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kEncoderConversionFactor =(kWheelCircumference) / kDriveGearRatio;
    public static final boolean kGyroReversed = false;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final PIDController headingPID = new PIDController(3, 0, 0);

    public static final double ksVolts = 0.3;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kPDriveVel = 4;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final ChassisSpeeds kChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      2,
      2,
      Math.PI / 2.0, 
      Rotation2d.fromDegrees(45));

  /*Arm Constants */
    public static final boolean kArmInverted = true;

    public static final int kArmCurrentLimit = 50;

    public static final double kArmGearRatio = 0.04;//Conversion Factior did not like doing math, 1/25 used here
    public static final double kArmPositionFactor = kArmGearRatio*2.0*Math.PI; 
    public static final double kArmVelocityFactor = kArmPositionFactor / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kArmVelocityFactor;
    public static final double karmV = 12/kArmFreeSpeed;
    public static final double kArmZeroCosineOffset = 1.342; // radians to add to converted arm position to get real-world arm position (starts at ~76.9deg angle)
    public static final PIDGains kArmPositionGains = new PIDGains(0.21, 0.0, 0.00);
    public static final ArmFeedforward kArmFeedforward =
      new ArmFeedforward(0, 0.9105, 2, 0.1);    
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = 
      new TrapezoidProfile.Constraints(.25, 2);

    public static final double kHomePosition = 0.0;
    public static final double kFeedPosition = 3.4;
    
  /*Intake Constants */
    public static final double kIntakeOutPOW =  0.35;
    public static final double kIntakeInPOW = -0.4;
    public static final int kIntakeCurrentLimit = 40;

  /*Vision Constants */
    public static final NetworkTable noteVision = inst.getTable("photonvision").getSubTable("Microsoft_LifeCam_HD-3000");
    public static final DoubleTopic noteYawTopic = noteVision.getDoubleTopic("targetYaw");
    public static final BooleanTopic colorHasTargetsTopic = noteVision.getBooleanTopic("hasTarget");
    public static final DoubleTopic notePitchTopic = noteVision.getDoubleTopic("targetPitch");
  
    
  
}

