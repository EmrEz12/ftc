package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
public class ArmBucket extends SequentialCommandGroup {
    /** Static factory for an autonomous command. */
    public ArmBucket(ArmSubsystem m_arm) {
        addCommands(
                new ArmCommand(m_arm, 0.9, 1),
                new ArmCommand(m_arm, 0.9, 2),
                new WristUp(m_arm),
                new ArmEject(m_arm)
        );
        addRequirements(m_arm);
    }
}