package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

//import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class WristMid extends CommandBase {

    private final ArmSubsystem m_subsytem;
    //private final Intake m_intake;
    public WristMid(ArmSubsystem subsystem){
        m_subsytem = subsystem;
        //m_intake = intake;


    }

    @Override
    public void initialize(){
        m_subsytem.wristmid();
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return false;
    }
}
