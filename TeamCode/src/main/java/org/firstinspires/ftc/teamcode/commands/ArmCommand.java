package org.firstinspires.ftc.teamcode.commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;


public class ArmCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_arm;
    private int armTrack;
    private double pos;
    private double gearRatio;
    private double trackLength;
    private double goal;
    private double rotationGoal;
    private double rotationError;


    /**
     * Creates a new SlideCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmCommand(ArmSubsystem subsystem, double position, int track) {
        m_arm = subsystem;
        armTrack = track;
        pos = position;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(armTrack == 1) {
            trackLength = ModifyingConstants.SLIDE_IN_TRACK_LENGTH; //Shoulder track length
            gearRatio = ModifyingConstants.SLIDE_IN_GEAR_RATIO; //Shoulder gear ratio
        }
        else if (armTrack == 2) {
            trackLength = ModifyingConstants.SLIDE_OUT_TRACK_LENGTH; //Extension track length
            gearRatio = ModifyingConstants.SLIDE_OUT_GEAR_RATIO; //Extension gear ratio
        }

        goal = trackLength * pos;
        rotationGoal = m_arm.lengthToRotations(goal, gearRatio);
    }


  /*else if (intakeTrack == 2) {
      trackAngle = ModifyingConstants.SLIDE_OUT_TRACK_LENGTH;
      gearRatio = ModifyingConstants.SLIDE_OUT_GEAR_RATIO;
    }*/

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rotationError = (rotationGoal + m_arm.getMotorRotation(armTrack));
        m_arm.ArmMove(m_arm.pidCalc(rotationError), armTrack);
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
        return (Math.abs(rotationError) <= 0.2); // Might need adjusting
    }
}
