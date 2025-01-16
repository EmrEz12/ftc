package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.lib.Command;
import org.firstinspires.ftc.teamcode.lib.CommandSequence;
import org.firstinspires.ftc.teamcode.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.util.Mechanism;

@Config
public class Scoring extends Mechanism {
    private Drivetrain drivetrain = new Drivetrain(opMode); // OPMODE NULL HERE
    public ArmSubsystem arm = new ArmSubsystem(opMode);
    public IntakeSubsystem intake = new IntakeSubsystem(opMode);
    public WristSubsystem wrist = new WristSubsystem(opMode);
    public ShoulderSubsystem shoulder = new ShoulderSubsystem(opMode);

    private State state = State.INTAKE;

    private enum State {
        OPENINTAKE,
        INTAKE,
        WALL,
        BASKET,
        CLIP,
    }



    private boolean frontClicked = false;
    private boolean dpadClicked = false;
    private boolean rightStickClicked = false;

    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Command release = () -> intake.outtake();
    private Command intaking = () -> intake.intake();
    private Command intStop = () -> intake.intstop();
    private Command shGround = () -> shoulder.shground();
    private Command shHang = () -> shoulder.shhang();
    private Command shWall = () -> shoulder.shwall();
    private Command shBucket = () -> shoulder.shbucket();
    private Command shopenintake = () -> shoulder.shopenintake();
    private Command armFront = () -> arm.intextended();
    private Command armWall = () -> arm.wall();
    private Command armDownIntake = () -> arm.ground();
    private Command armHang = () -> arm.hang();
    private Command armBucket = () -> arm.bucket();
    private Command setStateOpenIntake = () -> state = State.OPENINTAKE;
    private Command setStateWall = () -> state = State.WALL;
    private Command setStateBasket = () -> state = State.BASKET;
    private Command setStateIntake = () -> state = State.INTAKE;
    //private Command setStateUpClip = () -> state = State.UPCLIP;
    //private Command setStateLowClip = () -> state = State.LOWERCLIP;
    private Command wristUp = () -> wrist.rotateUp();
    private Command wristMid = () -> wrist.rotateMid();
    private Command wristOrigin = () -> wrist.rotateOrigin();
    private Command grabOpen = () -> wrist.HandOpen();
    private Command grabClose = () -> wrist.HandClose();

    private CommandSequence wallIntake = new CommandSequence()
            .addCommand(armWall)
            .addCommand(wristMid)
            .build();

    private CommandSequence scoreBasket = new CommandSequence()
            .addCommand(shBucket)
            .addCommand(armBucket)
            .addCommand(wristUp)
            .build();

    private CommandSequence scoreClip = new CommandSequence()
            .addCommand(shHang)
            .addCommand(armWall)
            .addCommand(wristMid)
            .build();

    public CommandSequence frontIntake = new CommandSequence()
            .addCommand(shopenintake)
            .addCommand(armFront)
            .addCommand(wristOrigin)
            .build();

    public CommandSequence frontIntakeShort = new CommandSequence()
            .addCommand(armDownIntake)
            .addCommand(wristOrigin)
            .addCommand(shGround)
            .build();


    public void goOpenIntake() {
        state = State.OPENINTAKE;
        shoulder.shopenintake();
        wrist.rotateOrigin();
        arm.intextended();
    }
    public void goIntake() {
        state = State.INTAKE;
        shoulder.shground();
        wrist.rotateOrigin();
        arm.ground();
    }

    public void goWall() {
        state = State.WALL;
        shoulder.shwall();
        wrist.rotateMid();
        arm.wall();
    }

    public void goBasket() {
        state = State.BASKET;
        shoulder.shbucket();
        arm.bucket();
        wrist.rotateUp();
    }

    public void goClip() {
        state = State.CLIP;
        shoulder.shhang();
        wrist.rotateMid();
        arm.hang();

    }

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        shoulder.init(hwMap);
        arm.init(hwMap);
        intake.init(hwMap);
        wrist.init(hwMap);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("state", state);
        telemetry.update();
    }

    @Override
    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        drivetrain.loop(gamepad1);

        if (GamepadStatic.isButtonPressed(gamepad2, Controls.GROUND) && state != State.INTAKE) {
            goIntake();
        } else if (GamepadStatic.isButtonPressed(gamepad2, Controls.WALL) && state != State.WALL) {
            goWall();
        } else if (GamepadStatic.isButtonPressed(gamepad2, Controls.BASKET) && state != State.BASKET) {
            goBasket();
        } else if (GamepadStatic.isButtonPressed(gamepad2, Controls.HANG) && state != State.CLIP) {
            goClip();
        }


        switch (state) {
            case BASKET:
                wrist.loop(gamepad2);
                drivetrain.setNormal();
                arm.loop(gamepad2);
                shoulder.loop(gamepad2);
                intake.loop(gamepad2);

            case WALL:
                wrist.loop(gamepad2);
                drivetrain.setNormal();
                arm.loop(gamepad2);
                shoulder.loop(gamepad2);
                intake.loop(gamepad2);

            case INTAKE:
                wrist.loop(gamepad2);
                drivetrain.setIntake();
                arm.loop(gamepad2);
                shoulder.loop(gamepad2);
                intake.loop(gamepad2);

            case CLIP:
                wrist.loop(gamepad2);
                drivetrain.setNormal();
                arm.loop(gamepad2);
                shoulder.loop(gamepad2);
                intake.loop(gamepad2);


        }
    }
}