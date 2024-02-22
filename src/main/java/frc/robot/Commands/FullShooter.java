package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FullShooter extends SequentialCommandGroup{
    public static boolean isFinished;

    public FullShooter(Intake intake, Shooter shooter){
        addCommands(
            new InstantCommand(() -> shooter.activeShooter()),
            new WaitCommand(2),
            new InstantCommand(() -> intake.IntakeShoot()),
            new WaitCommand(2),
            new InstantCommand(() -> shooter.stopShooter()),
            new InstantCommand(() -> intake.IntakeStop())
        );
    }
}