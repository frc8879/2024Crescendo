// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.AutoRoutines.AutoSnL;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climbers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.FullPickup;
import frc.robot.Commands.FullShooter;
import frc.robot.Commands.IntakeHardStop; 


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
     // set up arm preset positions
    operator.b()
      .onTrue(new FullPickup(intake, arm, driveTrain));
    operator.a()
      .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.kFeedPosition)));
    operator.y()
      .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.kHomePosition)));

    operator.x()
    .onTrue(new FullShooter(intake, shooter));
    

      
    driver.leftTrigger()
    .onTrue(new InstantCommand(() -> climbers.leftClimberDown()))
    .onFalse(new InstantCommand(() -> climbers.leftClimberStop()));

    driver.rightTrigger()
    .onTrue(new InstantCommand(() -> climbers.rightClimberDown()))
    .onFalse(new InstantCommand(() -> climbers.rightClimberStop()));
  
    driver.rightBumper()
    .onTrue(new InstantCommand(() -> climbers.rightClimberUp()))
    .onFalse (new InstantCommand(() -> climbers.rightClimberStop()));

    driver.leftBumper()
    .onTrue(new InstantCommand(() -> climbers.leftClimberUp()))
    .onFalse(new InstantCommand(()-> climbers.leftClimberStop()));
  }
  



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoSnL(driveTrain, intake, shooter);
  }
}

