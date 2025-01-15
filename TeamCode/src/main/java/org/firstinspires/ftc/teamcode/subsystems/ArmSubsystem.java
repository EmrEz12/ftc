package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.lib.Command;
import org.firstinspires.ftc.teamcode.lib.CommandSequence;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

public class ArmSubsystem extends Mechanism {
    private static double P = 0.00065;
    private static double I = 0.0;
    private static double D = 0.0;
    private static double F = 0.0;

    private static double target = 0;
    private static double actualTarget = 0;
    private static double power = 0;
    private PIDFController pidcontroller;
    private DcMotorEx Motor1EXT;
    private DcMotorEx Motor22ndEXT;

    private VoltageSensor voltage;

    private Command resetEncoders = () -> {
        Motor1EXT.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Motor22ndEXT.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //teleOp.Motor0SHLDR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Motor1EXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
        Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
        //teleOp.Motor0SHLDR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    };

    private CommandSequence waitToReset = new CommandSequence()
            .addWaitCommand(1)
            .addCommand(resetEncoders)
            .build();

    public ArmSubsystem(LinearOpMode opMode){
        this.opMode = opMode;

    }

    @Override
    public void init(HardwareMap hwMap) {

        Motor1EXT = hwMap.get(DcMotorEx.class, "Motor-1-EXT");
        Motor22ndEXT = hwMap.get(DcMotorEx.class, "Motor-2-2ndEXT");
        ground();

        Motor1EXT.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Motor22ndEXT.setMode(RunMode.STOP_AND_RESET_ENCODER);
        //teleOp.Motor0SHLDR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);

        //teleOp.Motor0SHLDR.setMode(RunMode.RUN_WITHOUT_ENCODER);

        //Motor1EXT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Motor22ndEXT.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //teleOp.Motor0SHLDR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Motor1EXT.setDirection(DcMotor.Direction.REVERSE);
        Motor22ndEXT.setDirection(DcMotor.Direction.FORWARD);
        //teleOp.Motor0SHLDR.setDirection(DcMotor.Direction.REVERSE);

        //teleOp.Motor0SHLDR.setPower(1);


    }

    public void telemetry(Telemetry telemetry) {
        //telemetry.addData("Current Position", getPosition());
        // telemetry.addData("Target", target);
        // telemetry.addData("Power", power);
        telemetry.update();
    }

    public void ground() {
        Motor22ndEXT.setPower(1);
        Motor1EXT.setPower(1);
        Motor1EXT.setTargetPosition(0);
        Motor22ndEXT.setTargetPosition(0);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        while (Motor22ndEXT.isBusy() || Motor1EXT.isBusy()){
            opMode.idle();
        }
    }

    public void intextended(){
        Motor22ndEXT.setPower(1);
        Motor1EXT.setPower(1);
        Motor1EXT.setTargetPosition(400);
        Motor22ndEXT.setTargetPosition(400);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        while (Motor22ndEXT.isBusy() || Motor1EXT.isBusy()){
            opMode.idle();
        }
    }
    public void wall() {
        Motor22ndEXT.setPower(1);
        Motor1EXT.setPower(1);
        Motor1EXT.setTargetPosition(400);
        Motor22ndEXT.setTargetPosition(400);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        while (Motor22ndEXT.isBusy() || Motor1EXT.isBusy()){
            opMode.idle();
        }
    }
    public void hang() {
        Motor1EXT.setTargetPosition(370);
        Motor22ndEXT.setTargetPosition(370);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        while (Motor22ndEXT.isBusy() || Motor1EXT.isBusy()){
            opMode.idle();
        }
    }
    public void bucket() {
        Motor22ndEXT.setPower(1);
        Motor1EXT.setPower(1);
        Motor1EXT.setTargetPosition(5000);
        Motor22ndEXT.setTargetPosition(5000);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        Motor22ndEXT.setMode(RunMode.RUN_TO_POSITION);
        while (Motor22ndEXT.isBusy() || Motor1EXT.isBusy()){
            opMode.idle();
        }
    }

    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.Ext)) {
            Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor1EXT.setPower(1);
            Motor22ndEXT.setPower(1);
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.ExtClose)) {
            Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor1EXT.setPower(-1);
            Motor22ndEXT.setPower(-1);
        } else {
            Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor22ndEXT.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor1EXT.setPower(0);
            Motor22ndEXT.setPower(0);
        }

        if (GamepadStatic.isButtonPressed(gamepad, Controls.Reset)){
            waitToReset.trigger();
        }

    }
}
