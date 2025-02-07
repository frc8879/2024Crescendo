package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake;

public class IntakeHardStop extends Command {
    Intake m_intake;

    /** Creates a new command named IntakeHardStop **/
    public IntakeHardStop(Intake m_intake){
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    //Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.IntakeFeed();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        m_intake.IntakeStop();
    }

    //Returns true when the command should end (return false for default)
    @Override
    public boolean isFinished(){
        return m_intake.isNoteIn();
        
    }

}
