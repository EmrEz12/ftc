package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
public class ArmWall extends SequentialCommandGroup {
    /** Static factory for an autonomous command. */
    public ArmWall(ArmSubsystem m_arm) {
        addCommands(
                new WristMid(m_arm),
                new ArmCommand(m_arm, 0.30, 1),
                new ArmOpen(m_arm),
                new ArmCommand(m_arm, 0.40, 2),
                new ArmClose(m_arm)
        );
        addRequirements(m_arm);
    }
}