package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class FullPickup extends SequentialCommandGroup{
    public static boolean isFinished;

    public FullPickup(Intake intake, Arm arm, DriveTrain drivetrain){
        addCommands(
            new InstantCommand(() -> arm.setTargetPosition(Constants.kFeedPosition)),
            new InstantCommand(() -> drivetrain.arcadeDrive(.25, 0)),
            new IntakeHardStop(intake),
            new InstantCommand(() -> drivetrain.arcadeDrive(0, 0)),
            new InstantCommand(() -> arm.setTargetPosition(Constants.kHomePosition))
        );
    }
}
