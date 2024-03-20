// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climbers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    //set drivetrain subsystem default to our drive command
    driveTrain.setDefaultCommand(new RunCommand(() -> {
      driveTrain.arcadeDrive(-driver.getLeftY(), -driver.getRightX());},driveTrain)); 
    
    // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running
    arm.setDefaultCommand(new RunCommand(() -> {
      arm.runAutomatic();}, arm));
    
    //Register Named Commands
    NamedCommands.registerCommand("FullShooter", new FullShooter(intake, shooter));
    NamedCommands.registerCommand("FullPickup", new FullPickup(intake, arm));
    NamedCommands.registerCommand("NoteAlign", new NoteAlign(driveTrain));

    autoChooser = AutoBuilder.buildAutoChooser("LongSideAuto");
    new PathPlannerAuto("LongSideAuto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
      .onTrue(new FullPickup(intake, arm));
      
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
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}


