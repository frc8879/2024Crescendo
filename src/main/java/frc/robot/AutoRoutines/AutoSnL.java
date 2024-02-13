package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;



//Autonoumous routine for SHOOTING n LEAVING
public class AutoSnL extends SequentialCommandGroup {


    public AutoSnL (DriveTrain drive, Intake intake, Shooter shooter){
        super(
            new WaitCommand(1),
            new InstantCommand(()-> intake.IntakeShoot(), intake),
            new InstantCommand(() -> shooter.activeShooter()),
            new WaitCommand(.5),
            new InstantCommand(() -> shooter.stopShooter()),
            new InstantCommand(() -> intake.IntakeStop(), intake)
            );
            
    }
}
