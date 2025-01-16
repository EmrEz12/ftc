package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;
import org.firstinspires.ftc.teamcode.input.GamepadStatic;
import org.firstinspires.ftc.teamcode.util.Mechanism;

@Config
public class IntakeSubsystem extends Mechanism {
    public static boolean outtaked = false;
    public static boolean intaked = false;
    private CRServo servo2;
    private CRServo servo3;


    public IntakeSubsystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void intake() {
        servo3.setPower(1);
        servo2.setPower(-1);
        intaked = true;
        outtaked = false;
    }

    public void outtake() {
        servo3.setPower(-1);
        servo2.setPower(1);
        outtaked = true;
        intaked = false;
    }

    public void intstop() {
        servo3.setPower(0);
        servo2.setPower(0);
        outtaked = false;
        intaked = false;
    }


    @Override
    public void init(HardwareMap hwMap) {
        servo2 = hwMap.get(CRServo.class, "servo2");
        servo3 = hwMap.get(CRServo.class, "servo3");
        servo2.setDirection(CRServo.Direction.FORWARD);
        servo3.setDirection(CRServo.Direction.FORWARD);

    }

    @Override
    public void loop(Gamepad gamepad2) {
        if (GamepadStatic.isButtonPressed(gamepad2, Controls.INTAKE)) {
            intake();
        } else if (GamepadStatic.isButtonPressed(gamepad2, Controls.OUTTAKE)) {
            outtake();
        } else {
            intstop();
        }
    }
}