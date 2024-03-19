package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class FullPickup extends SequentialCommandGroup{
    public static boolean isFinished;

    public FullPickup(Intake intake, Arm arm){
        addCommands(
            new InstantCommand(() -> arm.setTargetPosition(Constants.kFeedPosition)),
            new IntakeHardStop(intake).withTimeout(4),
            new InstantCommand(() -> arm.setTargetPosition(Constants.kHomePosition))
        );
    }
}
