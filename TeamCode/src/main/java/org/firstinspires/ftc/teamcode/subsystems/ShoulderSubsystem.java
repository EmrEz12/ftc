package org.firstinspires.ftc.teamcode.subsystems;
import         org.firstinspires.ftc.teamcode.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.lib.Command;
import org.firstinspires.ftc.teamcode.lib.CommandSequence;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class ShoulderSubsystem extends Mechanism {
    private static double KP = 0.00065;
    private static double KI = 0.05;
    private static double KD = 0.00005;
    private static double KF = 0.005;
    private static double HIGHEST = 8000;

    private PIDFController pidcontroller;
    private DcMotorEx Motor0SHLDR;


    private Command resetEncoders = () -> {
        Motor0SHLDR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Motor0SHLDR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    };

    private CommandSequence waitToReset = new CommandSequence()
            .addWaitCommand(1)
            .addCommand(resetEncoders)
            .build();

    public ShoulderSubsystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        Motor0SHLDR = hwMap.get(DcMotorEx.class, "Motor-0-SHLDR");

        Motor0SHLDR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        shground();
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        Motor0SHLDR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Motor0SHLDR.setDirection(DcMotor.Direction.REVERSE);

    }

    public void telemetry(Telemetry telemetry) {
        // telemetry.addData("Current Position", getPosition());
        // telemetry.addData("Target", target);
        // telemetry.addData("Power", power);
        telemetry.update();
    }

    public void shground(){
        Motor0SHLDR.setTargetPosition(0);
        Motor0SHLDR.setPower(1);
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        if(Motor0SHLDR.isBusy()){
            Motor0SHLDR.getCurrentPosition();
        } else {
            Motor0SHLDR.setPower(0);
        }
    }
    public void shfront(){
        Motor0SHLDR.setTargetPosition(1000);
        Motor0SHLDR.setPower(1);
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        if(Motor0SHLDR.isBusy()){
            Motor0SHLDR.getCurrentPosition();
        } else {
            Motor0SHLDR.setPower(0);
        }
    }
    public void shopenintake(){
        Motor0SHLDR.setTargetPosition(800);
        Motor0SHLDR.setPower(1);
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        if(Motor0SHLDR.isBusy()){
            Motor0SHLDR.getCurrentPosition();
        } else {
            Motor0SHLDR.setPower(0);
        }
    }

    public void shwall(){
        Motor0SHLDR.setTargetPosition(1000);
        Motor0SHLDR.setPower(1);
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        if(Motor0SHLDR.isBusy()){
            Motor0SHLDR.getCurrentPosition();
        } else {
            Motor0SHLDR.setPower(0);
        }
    }
    public void shhang(){
        Motor0SHLDR.setTargetPosition(1300);
        Motor0SHLDR.setPower(1);
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        if(Motor0SHLDR.isBusy()){
            Motor0SHLDR.getCurrentPosition();
        } else {
            Motor0SHLDR.setPower(0);
        }
    }
    public void shbucket(){

        Motor0SHLDR.setTargetPosition(5600);
        Motor0SHLDR.setPower(1);
        Motor0SHLDR.setMode(RunMode.RUN_TO_POSITION);
        if(Motor0SHLDR.isBusy()){
            Motor0SHLDR.getCurrentPosition();
        } else {
            Motor0SHLDR.setPower(0);
        }
    }


    public double getPosition() {
        return Motor0SHLDR.getCurrentPosition();
    }


    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.SHUP)) {
            Motor0SHLDR.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor0SHLDR.setPower(1);
        } else if (GamepadStatic.isButtonPressed(gamepad, Controls.SHDOWN)) {
            Motor0SHLDR.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor0SHLDR.setPower(-0.9);
        } else {
            Motor0SHLDR.setMode(RunMode.RUN_WITHOUT_ENCODER);
            Motor0SHLDR.setPower(0);
        }
    }
}