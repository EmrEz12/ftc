// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmComManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_arm;
  private int intakeTrack;
  private double dir;

  /**
   * Creates a new SlideCommandManual.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmComManual(ArmSubsystem subsystem, double direction, int track) {
    m_arm = subsystem;
    intakeTrack = track;
    dir = Math.abs(direction)/direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeTrack == 1) {
      m_arm.ArmMove(dir , intakeTrack);
    }
    else if (intakeTrack == 2) {
      m_arm.ArmMove(dir , intakeTrack);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.ArmMove(0, 1);
    m_arm.ArmMove(0, 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}