package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;



//Autonoumous routine for SHOOTING n LEAVING
public class ShootAndLeave extends SequentialCommandGroup {


    public ShootAndLeave (Intake intake, Shooter shooter){
        super(
            new InstantCommand(() -> shooter.activeShooter()),
            new WaitCommand(1),
            new InstantCommand(()-> intake.IntakeShoot(), intake),
            new WaitCommand(.5),
            new InstantCommand(() -> shooter.stopShooter()),
            new InstantCommand(() -> intake.IntakeStop(), intake)
            );  
    }
}
