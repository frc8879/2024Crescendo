// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  /*IO Constants */
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

  /*CAN BUS IDs */
    public static final int LEFT_LEAD_ID = 3;
    public static final int RIGHT_LEAD_ID = 1;
    public static final int LEFT_FOLLOW_ID = 4;
    public static final int RIGHT_FOLLOW_ID = 2;

    public static final int GYRO_ID = 5;

    public static final int kArmMotor_ID = 6;
    public static final int kIntakeMotor_ID = 7;
    public static String CANBUS_NAME = "rio";
    public static final int kRightShooterMotor_ID = 8;
    public static final int kLeftShooterMotor_ID = 9;
  /*Drivetrain Constants */
    public static final double kDriveGearRatio = 6.86; //2 REV-21-1650s connected to single output
    public static final int    kDriveEncoderCPR = 42; //used Hall-Sensor Encoder Resolution from data sheet for REV-21-1650
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelCircumference = kWheelDiameterInches * Math.PI;// actual equation is gearRatio * circumference of wheel in meters. Check math
    public static final double kEncoderConversionFactor =(kWheelDiameterInches * Math.PI) * kDriveGearRatio;
    public static final boolean kGyroReversed = false;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

  /*Arm Constants */
    public static final boolean kArmInverted = true;

    public static final int kCurrentLimit = 50;

    public static final double kArmGearRatio = 0.04;//Conversion Factior did not like doing math, 1/25 used here
    public static final double kPositionFactor = kArmGearRatio* 2.0* Math.PI; // multiply SM value by this number and get arm position in radians
    public static final double kArmVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kArmVelocityFactor;
    public static final double kArmZeroCosineOffset = 1.342; // radians to add to converted arm position to get real-world arm position (starts at ~76.9deg angle)
    public static final ArmFeedforward kArmFeedforward =
        new ArmFeedforward(0.0005, 2.0, .50 / kArmFreeSpeed, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(3, 0.00, 0.1);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint =
        new TrapezoidProfile.Constraints(.03, .02);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = .3;
    public static final double kFeedPosition = 0.54;

  /*Intake Constants */
  public static final double kIntake_Out_POW =  1;
  public static final double kIntake_In_POW = 1;
  public static final int kIntake_Current_Limit = 30;
  public static final int kIntake_Hold_Current_Limit = 10;


  
}

