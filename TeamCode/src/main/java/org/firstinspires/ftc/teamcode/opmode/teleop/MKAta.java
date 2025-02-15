package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp M.K.Atatürk")
public class MKAta extends LinearOpMode {

  private DcMotor Motor0SHLDR;
  private DcMotor Motor1EXT;
  private DcMotor Motor0FL;
  private DcMotor Motor1FR;
  private DcMotor Motor2LEXT;
  private DcMotor Motor2BL;
  private DcMotor Motor3BR;
  private DcMotor Motor3LSH;
  private CRServo Servo0Wrist;
  private CRServo servo1;
  private CRServo servo2;
  private CRServo servo3;
  private Servo GripSpin;
  private Servo SubGrip;

  double maxDrivePower;

  /**
   * Describe this function...
   */
  private void Arm_PID(int shouldertarget, int exttarget) {
    // Experimental PID code
    // I dont wanna explain this.
    if (Motor0SHLDR.getCurrentPosition() < shouldertarget) {
      Motor0SHLDR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Motor1EXT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Motor1EXT.setPower(0);
      Motor0SHLDR.setTargetPosition(Motor0SHLDR.getCurrentPosition() + 500);
      Motor0SHLDR.setPower(1);
      while (opModeIsActive() && Motor0SHLDR.isBusy()) {
        Mecanum_Drive();
        Head();
        telemetry2();
        if (gamepad1.back) {
          break;
        }
      }
      Motor0SHLDR.setTargetPosition(shouldertarget - 500);
      Motor1EXT.setTargetPosition(-exttarget);
      Motor1EXT.setPower(1);
      while (opModeIsActive() && (Motor0SHLDR.isBusy() || Motor1EXT.isBusy())) {
        Mecanum_Drive();
        Head();
        telemetry2();
        if (gamepad1.back) {
          break;
        }
      }
      Motor0SHLDR.setTargetPosition(shouldertarget);
      while (opModeIsActive() && (Motor0SHLDR.isBusy() || Motor1EXT.isBusy())) {
        Mecanum_Drive();
        Head();
        telemetry2();
        if (gamepad1.back) {
          break;
        }
      }
      Motor0SHLDR.setPower(0);
      Motor1EXT.setPower(0);
      Motor0SHLDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      Motor1EXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    } else {
      Motor0SHLDR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Motor1EXT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Motor1EXT.setPower(0);
      Motor0SHLDR.setTargetPosition(Motor0SHLDR.getCurrentPosition() - 500);
      Motor0SHLDR.setPower(1);
      while (opModeIsActive() && Motor0SHLDR.isBusy()) {
        Mecanum_Drive();
        Head();
        telemetry2();
        if (gamepad1.back) {
          break;
        }
      }
      Motor0SHLDR.setTargetPosition(shouldertarget + 500);
      Motor1EXT.setTargetPosition(-exttarget);
      Motor1EXT.setPower(1);
      while (opModeIsActive() && (Motor0SHLDR.isBusy() || Motor1EXT.isBusy())) {
        Mecanum_Drive();
        Head();
        telemetry2();
        if (gamepad1.back) {
          break;
        }
      }
      Motor0SHLDR.setTargetPosition(shouldertarget);
      while (opModeIsActive() && (Motor0SHLDR.isBusy() || Motor1EXT.isBusy())) {
        Mecanum_Drive();
        Head();
        telemetry2();
        if (gamepad1.back) {
          break;
        }
      }
      Motor0SHLDR.setPower(0);
      Motor1EXT.setPower(0);
      Motor0SHLDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      Motor1EXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
  }

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    Motor0SHLDR = hardwareMap.get(DcMotor.class, "Motor-0-SHLDR");
    Motor1EXT = hardwareMap.get(DcMotor.class, "Motor-1-EXT");
    Motor0FL = hardwareMap.get(DcMotor.class, "Motor-0-FL");
    Motor1FR = hardwareMap.get(DcMotor.class, "Motor-1-FR");
    Motor2LEXT = hardwareMap.get(DcMotor.class, "Motor-2-LEXT");
    Motor2BL = hardwareMap.get(DcMotor.class, "Motor-2-BL");
    Motor3BR = hardwareMap.get(DcMotor.class, "Motor-3-BR");
    Motor3LSH = hardwareMap.get(DcMotor.class, "Motor-3-LSH");
    Servo0Wrist = hardwareMap.get(CRServo.class, "Servo-0-Wrist");
    servo1 = hardwareMap.get(CRServo.class, "servo1");
    servo2 = hardwareMap.get(CRServo.class, "servo2");
    servo3 = hardwareMap.get(CRServo.class, "servo3");
    GripSpin = hardwareMap.get(Servo.class, "GripSpin");
    SubGrip = hardwareMap.get(Servo.class, "SubGrip");

    // Put initialization blocks here.
    ((DcMotorEx) Motor0FL).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor0SHLDR).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor1EXT).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor1FR).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor2LEXT).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor2BL).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor3BR).setTargetPositionTolerance(50);
    ((DcMotorEx) Motor3LSH).setTargetPositionTolerance(50);
    Motor1FR.setDirection(DcMotor.Direction.REVERSE);
    Motor3BR.setDirection(DcMotor.Direction.REVERSE);
    Motor1EXT.setDirection(DcMotor.Direction.FORWARD);
    Motor2LEXT.setDirection(DcMotor.Direction.REVERSE);
    Motor0SHLDR.setDirection(DcMotor.Direction.REVERSE);
    Motor0FL.setDirection(DcMotor.Direction.FORWARD);
    Motor2BL.setDirection(DcMotor.Direction.FORWARD);
    Servo0Wrist.setDirection(CRServo.Direction.FORWARD);
    servo1.setDirection(CRServo.Direction.REVERSE);
    servo2.setDirection(CRServo.Direction.FORWARD);
    servo3.setDirection(CRServo.Direction.FORWARD);
    Motor0SHLDR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor1EXT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor2LEXT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Motor0SHLDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Motor1EXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Motor2LEXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Motor1EXT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    maxDrivePower = 0.7;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        Mecanum_Drive();
        telemetry2();
        // Arm Systems
        if (gamepad2.dpad_left) {
          Motor1EXT.setPower(1);
        } else if (gamepad2.dpad_right) {
          Motor1EXT.setPower(-1);
        } else {
          Motor1EXT.setPower(0);
        }
        if (gamepad2.dpad_up) {
          Motor0SHLDR.setPower(maxDrivePower);
        } else if (gamepad2.dpad_down) {
          Motor0SHLDR.setPower(-maxDrivePower);
        } else {
          Motor0SHLDR.setPower(0);
        }
        Head();
        // Automated Actions
        // All positions are place-holders as of 29DEC24
        if (gamepad1.a || gamepad2.a) {
          // Intake From Floor
          if (Motor0SHLDR.getCurrentPosition() < 850) {
            Servo0Wrist.setPower(0.05);
            Arm_PID(1000, 0);
            Arm_PID(0, 0);
          } else {
            Servo0Wrist.setPower(0.05);
            Arm_PID(0, 0);
          }
        }
        if (gamepad1.y || gamepad2.y) {
          // Outtake High Bucket
          Servo0Wrist.setPower(0.62);
          Arm_PID(4800, 4800);
        }
        if (gamepad1.x || gamepad2.x) {
          // Outtake High Chamber
          Servo0Wrist.setPower(0.25);
          Arm_PID(3000, 1250);
        }
        if (gamepad1.right_stick_button || gamepad2.right_stick_button) {
          Servo0Wrist.setPower((Servo0Wrist.getPower() - gamepad2.right_trigger) / 2);
          Servo0Wrist.setPower((Servo0Wrist.getPower() - gamepad1.right_trigger) / 2);
        }
        // Arm Telemetry Reset (do at Intake position)
        if (gamepad1.start && gamepad1.back || gamepad2.start && gamepad2.back) {
          Motor0SHLDR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor1EXT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor2LEXT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          Motor3LSH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // LARM
        if (gamepad1.dpad_up) {
          Motor3LSH.setPower(0.4);
        } else if (gamepad1.dpad_down) {
          Motor3LSH.setPower(-0.4);
        } else {
          Motor3LSH.setPower(0);
        }
        if (gamepad1.dpad_right) {
          Motor2LEXT.setPower(0.7);
        } else if (gamepad1.dpad_left) {
          Motor2LEXT.setPower(-0.7);
        } else {
          Motor2LEXT.setPower(0);
        }
        GripSpin.setPosition(gamepad1.left_trigger);
        if (gamepad1.left_bumper) {
          SubGrip.setPosition(0.8);
        } else if (gamepad1.right_bumper) {
          SubGrip.setPosition(0.2);
        } else {
          SubGrip.setPosition(0.5);
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Mecanum_Drive() {
    // Mechanum Omnidirectional Drive
    if (gamepad1.left_stick_button) {
      Motor0FL.setPower(Math.pow(gamepad1.left_stick_y + (-gamepad1.left_stick_x - gamepad1.right_stick_x), 3) * 0.5);
      Motor1FR.setPower(Math.pow(gamepad1.left_stick_y - (-gamepad1.left_stick_x - gamepad1.right_stick_x), 3) * 0.5);
      Motor2BL.setPower(Math.pow(gamepad1.left_stick_y - (-gamepad1.left_stick_x + gamepad1.right_stick_x), 3) * 0.5);
      Motor3BR.setPower(Math.pow(gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x, 3) * 0.5);
    } else {
      Motor0FL.setPower(Math.pow(gamepad1.left_stick_y + (-gamepad1.left_stick_x - gamepad1.right_stick_x), 5) * maxDrivePower);
      Motor1FR.setPower(Math.pow(gamepad1.left_stick_y - (-gamepad1.left_stick_x - gamepad1.right_stick_x), 5) * maxDrivePower);
      Motor2BL.setPower(Math.pow(gamepad1.left_stick_y - (-gamepad1.left_stick_x + gamepad1.right_stick_x), 5) * maxDrivePower);
      Motor3BR.setPower(Math.pow(gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x, 5) * maxDrivePower);
    }
  }

  /**
   * Describe this function...
   */
  private void Head() {
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
    servo1.setPower((gamepad2.left_trigger - 0.1) / 2);
    servo1.setPower((gamepad1.left_trigger - 0.1) / 2);
  }

  /**
   * Describe this function...
   */
  private void telemetry2() {
    telemetry.update();
    telemetry.addData("USH", Motor0SHLDR.getCurrentPosition());
    telemetry.addData("UEXT", Motor1EXT.getCurrentPosition());
    telemetry.addData("LEXT", Motor2LEXT.getCurrentPosition());
    telemetry.addData("LSH", Motor3LSH.getCurrentPosition());
  }
}
