package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.ArmBucket;
import org.firstinspires.ftc.teamcode.commands.ArmWall;
import org.firstinspires.ftc.teamcode.commands.WristMid;
import org.firstinspires.ftc.teamcode.commands.WristOrigin;
import org.firstinspires.ftc.teamcode.commands.WristUp;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import org.firstinspires.ftc.teamcode.commands.ArmGround;

@TeleOp(name = "TeleOpTwoControllersTikiTechATA (M.K.ATA)")
public class TeleOpTwoControllersTikiTechATA extends LinearOpMode {
  private ArmSubsystem m_arm;
  private DcMotor Motor1FR;
  private DcMotor Motor3BR;
  private DcMotor Motor1EXT;
  private DcMotor Motor22ndEXT;
  private DcMotor Motor0SHLDR;
  private DcMotor Motor0FL;
  private DcMotor Motor2BL;
  private CRServo Servo0WRIST;
  private CRServo servo1;
  private CRServo servo2;
  private CRServo servo3;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int maxDrivePower;

    Motor1FR = hardwareMap.get(DcMotor.class, "Motor-1-FR");
    Motor3BR = hardwareMap.get(DcMotor.class, "Motor-3-BR");
    Motor1EXT = hardwareMap.get(DcMotor.class, "Motor-1-EXT");
    Motor22ndEXT = hardwareMap.get(DcMotor.class, "Motor-2-2ndEXT");
    Motor0SHLDR = hardwareMap.get(DcMotor.class, "Motor-0-SHLDR");
    Motor0FL = hardwareMap.get(DcMotor.class, "Motor-0-FL");
    Motor2BL = hardwareMap.get(DcMotor.class, "Motor-2-BL");
    Servo0WRIST = hardwareMap.get(CRServo.class, "Servo-0-WRIST");
    servo1 = hardwareMap.get(CRServo.class, "servo1");
    servo2 = hardwareMap.get(CRServo.class, "servo2");
    servo3 = hardwareMap.get(CRServo.class, "servo3");

    // Put initialization blocks here.
    Motor1FR.setDirection(DcMotor.Direction.REVERSE);
    Motor3BR.setDirection(DcMotor.Direction.REVERSE);
    Motor1EXT.setDirection(DcMotor.Direction.FORWARD);
    Motor22ndEXT.setDirection(DcMotor.Direction.REVERSE);
    Motor0SHLDR.setDirection(DcMotor.Direction.REVERSE);
    Motor0FL.setDirection(DcMotor.Direction.FORWARD);
    Motor2BL.setDirection(DcMotor.Direction.FORWARD);
    Servo0WRIST.setDirection(CRServo.Direction.FORWARD);
    servo1.setDirection(CRServo.Direction.FORWARD);
    servo2.setDirection(CRServo.Direction.FORWARD);
    servo3.setDirection(CRServo.Direction.FORWARD);
    maxDrivePower = 1;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Mechanum Omnidirectional Drive
        Motor0FL.setPower(Math.pow(gamepad1.left_stick_y + (-gamepad1.left_stick_x - gamepad1.right_stick_x), 3) * maxDrivePower);
        Motor1FR.setPower(Math.pow(gamepad1.left_stick_y - (-gamepad1.left_stick_x - gamepad1.right_stick_x), 3) * maxDrivePower);
        Motor2BL.setPower(Math.pow(gamepad1.left_stick_y - (-gamepad1.left_stick_x + gamepad1.right_stick_x), 3) * maxDrivePower);
        Motor3BR.setPower(Math.pow(gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x, 3) * maxDrivePower);
        telemetry.update();
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
          Motor0SHLDR.setPower(1);
        } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
          Motor0SHLDR.setPower(-0.9);
        } else {
          Motor0SHLDR.setPower(0);
        }
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
          Motor1EXT.setPower(1);
          Motor22ndEXT.setPower(1);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
          Motor1EXT.setPower(-1);
          Motor22ndEXT.setPower(-1);
        } else {
          Motor1EXT.setPower(0);
          Motor22ndEXT.setPower(0);
        }
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
          servo3.setPower(1);
          servo2.setPower(-1);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
          servo3.setPower(-1);
          servo2.setPower(1);
        } else {
          servo3.setPower(0);
          servo2.setPower(0);
        }
        if (gamepad1.right_stick_button || gamepad2.right_stick_button) {
          new WristOrigin(m_arm);
        }
        if (gamepad2.right_trigger >= 0.5){
          new WristUp(m_arm);
        }
        if (gamepad2.left_trigger >= 0.5){
          new WristMid(m_arm);
        }
        // All wrist positions are place-holders as of 23DEC24
        // Intake from Wall
        if (gamepad1.y || gamepad2.y) {
          new ArmBucket(m_arm);
        }
        // Intake from Floor
        if (gamepad1.x || gamepad2.x) {
          new ArmWall(m_arm);
        }
        // Outtake in Bucket
        if (gamepad1.b || gamepad2.b) {
          servo2.setPower(1);
          servo3.setPower(-1);
        }
        // Outtake on Chamber
        if (gamepad1.a || gamepad2.a) {
          new ArmGround(m_arm);
        }
      }
    }
  }
}
