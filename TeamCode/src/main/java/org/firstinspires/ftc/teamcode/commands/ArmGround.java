package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
public class ArmGround extends ParallelCommandGroup {
    /** Static factory for an autonomous command. */
    public ArmGround(ArmSubsystem m_arm) {
        addCommands(
                new WristOrigin(m_arm),
                new ArmCommand(m_arm, 0.55, 1),
                new ArmCommand(m_arm, 0.55, 2),
                new ArmPulse(m_arm)
        );
        addRequirements(m_arm);
    }
}