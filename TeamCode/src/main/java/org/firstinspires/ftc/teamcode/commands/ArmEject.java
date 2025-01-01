package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

//import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class ArmEject extends CommandBase {

    private final ArmSubsystem m_subsytem;
    //private final Intake m_intake;
    public ArmEject(ArmSubsystem subsystem){
        m_subsytem = subsystem;
        //m_intake = intake;


    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_subsytem.eject();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsytem.intakestop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}