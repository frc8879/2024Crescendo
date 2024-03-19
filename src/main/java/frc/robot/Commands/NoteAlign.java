
package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class NoteAlign extends Command{
    DriveTrain m_driveTrain;
    DoubleSubscriber angleSubscriber;
    BooleanSubscriber hasTargetSubscriber;
    DoubleSubscriber pitchSubscriber;
    Rotation2d desiredRotation;
    Timer timer;
    boolean timingAlingment = false;

    public NoteAlign(DriveTrain m_driveTrain){
        this.m_driveTrain = m_driveTrain;

        angleSubscriber = Constants.noteYawTopic.subscribe(0.0);
        hasTargetSubscriber = Constants.colorHasTargetsTopic.subscribe(false);
        pitchSubscriber = Constants.notePitchTopic.subscribe(0.0);
        desiredRotation = m_driveTrain.getPose().getRotation();
        timer = new Timer();

        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(!hasTargetSubscriber.get()) return;

        Rotation2d robotAngle = m_driveTrain.getPose().getRotation();
        Rotation2d robotToNoteRotation = Rotation2d.fromDegrees(-angleSubscriber.get());
        desiredRotation = robotAngle.rotateBy(robotToNoteRotation);

        m_driveTrain.driveFacing(0,desiredRotation);
        
    }
}
