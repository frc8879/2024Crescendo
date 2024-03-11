// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.AutoRoutines.ShootAndLeave;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climbers;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.FullPickup;
import frc.robot.Commands.FullShooter;
import frc.robot.Commands.NoteAlign;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climbers climbers = new Climbers();
 
  private final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_PORT);
  private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_PORT);

  public RobotContainer() {
    //set drivetrain subsystem default to our drive command
    driveTrain.setDefaultCommand(new RunCommand(() -> {
      driveTrain.arcadeDrive(-driver.getLeftY(), -driver.getRightX());},driveTrain)); 
    
    // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
    arm.setDefaultCommand(new RunCommand(() -> {
      arm.runAutomatic();}, arm));
    
    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {  
    driver.a()
    .onTrue(new NoteAlign(driveTrain).withTimeout(.1));

    driver.rightTrigger()
    .onTrue(new InstantCommand(() -> climbers.rightClimberUp()))
    .onFalse(new InstantCommand(() -> climbers.rightClimberStop()));
  
    driver.rightBumper()
    .onTrue(new InstantCommand(() -> climbers.rightClimberDown()))
    .onFalse (new InstantCommand(() -> climbers.rightClimberStop()));

    driver.leftTrigger()
    .onTrue(new InstantCommand(() -> climbers.leftClimberUp()))
    .onFalse(new InstantCommand(() -> climbers.leftClimberStop()));
  
    driver.leftBumper()
    .onTrue(new InstantCommand(() -> climbers.leftClimberDown()))
    .onFalse (new InstantCommand(() -> climbers.leftClimberStop()));

    operator.a()
      .onTrue(new FullPickup(intake, arm, driveTrain));
      
    operator.b()
    .onTrue(new FullShooter(intake, shooter));
    
    operator.leftTrigger()
    .onTrue(new InstantCommand(() -> intake.IntakeFeed()))
    .onFalse(new InstantCommand(() -> intake.IntakeStop()));

    operator.rightTrigger()
    .onTrue(new InstantCommand(() -> intake.IntakeShoot()))
    .onFalse (new InstantCommand(() ->intake.IntakeStop()));

    operator.x()
    .onTrue(new InstantCommand(()->arm.goBack()).withTimeout(1));
     /*   USED FOR TESTING PURPOSES ONLY DO NOT UNCOMMENT
    operator.a()
        .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
        
    operator.b()
        .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
    operator.rightTrigger()
        .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        
    operator.leftTrigger()
        .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    */

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
    Trajectory firstTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
           List.of(new Translation2d(.25, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 2, new Rotation2d(20)),
            // Pass config
            config);
    
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            firstTrajectory,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain);
    return (
        new ShootAndLeave(intake,shooter))
        .andThen(Commands.runOnce(() -> driveTrain.resetOdometry(firstTrajectory.getInitialPose())))
        .andThen(ramseteCommand)  
        .andThen(Commands.runOnce(() -> driveTrain.tankDriveVolts(0, 0)));
  }
}


