package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@Config
public class WristSubsystem extends Mechanism {
    private CRServo Servo0WRIST;
    private CRServo servo1;

    public WristSubsystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        Servo0WRIST = hwMap.get(CRServo.class, "Servo-0-WRIST");
        servo1 = hwMap.get(CRServo.class, "servo1");
        Servo0WRIST.setDirection(CRServo.Direction.FORWARD);
        servo1.setDirection(CRServo.Direction.REVERSE);
        rotateOrigin();
    }

    public void rotateOrigin() {
        Servo0WRIST.setPower(0.05);
    }

    public void rotateMid() {
        Servo0WRIST.setPower(0.25);
    }

    public void rotateUp() {
        Servo0WRIST.setPower(0.62);
    }

    public void HandOpen(){
        servo1.setPower(0.55);
    }
    public void HandClose(){
        servo1.setPower(0.05);
    }


    @Override
    public void loop(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.OPEN)) {
            HandOpen();
        } else {
            HandClose();
        }

    }
}