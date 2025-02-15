package org.firstinspires.ftc.teamcode.opmode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "ClipAuto/M.K.Atatürk", preselectTeleOp = "TeleOp M.K.Atatürk")
public class Observation3Sample extends LinearOpMode {

    @Override
    public void runOpMode() {

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Pose2d initialPose = new Pose2d(8.75, -63.13, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        while (opModeIsActive()) {
            // FIRST SCORE WITH PRELOAD
            Actions.runBlocking(drive.actionBuilder(initialPose)
                    .afterTime(0.0, arm.closeArm())
                    .afterTime(0.0, arm.armUpSh())
                    .afterTime(1, arm.armUpExt())
                    .afterTime(2.6, arm.armMidSh())
                    .strafeToLinearHeading(new Vector2d(8.75, -41.12), Math.toRadians(90))
                    .build());

            // GO TO FIRST PICKUP AND PICKUP
            Actions.runBlocking(drive.actionBuilder(new Pose2d(8.75, -37.12, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(31,-46), Math.toRadians(30))
                    .afterTime(0, arm.openArm())
                    .afterTime(0.4, arm.armDownSh())
                    .afterTime(0.2, arm.armDownExt())
                    .afterTime(0.4, claw.clawUpSh())
                    .afterTime(2, claw.clawMid())
                    .afterTime(2.7, claw.clawmidDownSh())
                    .afterTime(3, claw.clawUpSh())
                    .build());

            // THROW HUMAN ZONE AND TURN FOR SECOND PICKUP
            Actions.runBlocking(drive.actionBuilder(new Pose2d(23, -41, Math.toRadians(30)))
                    .strafeToLinearHeading(new Vector2d(31,-46), Math.toRadians(330))
                    .afterTime(0.9, claw.openClaw())
                    .afterTime(1.2, claw.clawExt())
                    .strafeToLinearHeading(new Vector2d(31,-46), Math.toRadians(25))
                    .afterTime(2.2, claw.clawExtDownSh())
                    .afterTime(2.7, claw.clawUpSh())
                    .build());

            // SECOND THROW HUMAN AND CLOSE EXT
            Actions.runBlocking(drive.actionBuilder(new Pose2d(23, -41, Math.toRadians(25)))
                    .strafeToLinearHeading(new Vector2d(23,-41), Math.toRadians(330))
                    .afterTime(1, claw.openClaw())
                    .afterTime(1.3, claw.clawExtOrigin())
                    .build());

            // INTAKE FROM HUMAN ZONE AND GO BACK ARM UP
            Actions.runBlocking(drive.actionBuilder(new Pose2d(23, -41, Math.toRadians(330)))
                    .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270))
                    .afterTime(0.3, arm.intake())
                    .strafeToLinearHeading(new Vector2d(48, -59), Math.toRadians(270))
                    .afterTime(3.8, arm.closeArm())
                    .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270))
                    .afterTime(4.4, arm.intakeStop())
                    .afterTime(4.4, arm.armUpSh())
                    .afterTime(4.4, arm.armUpExt())
                    .build());

            // SCORE 2ND SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -47, Math.toRadians(-90)))
                    .strafeToLinearHeading(new Vector2d(7.65, -37.12), Math.toRadians(90))
                    .afterTime(4, arm.armMidSh())
                    .build());

            // RETURN HUMAN FOR 3RD SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(7.65, -37.12, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(48,-47), Math.toRadians(270))
                    .afterTime(0.6, arm.armDownExt())
                    .afterTime(0.6, arm.armDownSh())
                    .build());

            // TAKE 3RD SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -47, Math.toRadians(270)))
                    .afterTime(0.3, arm.intake())
                    .strafeToLinearHeading(new Vector2d(48, -59), Math.toRadians(270))
                    .afterTime(3.8, arm.closeArm())
                    .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270))
                    .afterTime(4.4, arm.intakeStop())
                    .afterTime(4.4, arm.armUpSh())
                    .afterTime(4.4, arm.armUpExt())
                    .build());

            // SCORE 3RD SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -47, Math.toRadians(270)))
                    .strafeToLinearHeading(new Vector2d(6.65, -37.12), Math.toRadians(90))
                    .afterTime(4, arm.armMidSh())
                    .build());

            // RETURN HUMAN FOR 4TH SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(6.65, -37.12, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(48,-47), Math.toRadians(270))
                    .afterTime(0.6, arm.armDownExt())
                    .afterTime(0.6, arm.armDownSh())
                    .build());

            // TAKE 4TH SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -47, Math.toRadians(270)))
                    .afterTime(0.3, arm.intake())
                    .strafeToLinearHeading(new Vector2d(48, -59), Math.toRadians(270))
                    .afterTime(3.8, arm.closeArm())
                    .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270))
                    .afterTime(4.4, arm.intakeStop())
                    .afterTime(4.4, arm.armUpSh())
                    .afterTime(4.4, arm.armUpExt())
                    .build());

            // SCORE 4TH SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(48, -47, Math.toRadians(270)))
                    .strafeToLinearHeading(new Vector2d(5.65, -37.12), Math.toRadians(90))
                    .afterTime(4, arm.armMidSh())
                    .build());

            // PARK
            Actions.runBlocking(drive.actionBuilder(new Pose2d(5.65, -37.12, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(53.92, -53.71), Math.toRadians(90))
                    .afterTime(0.5, arm.armDownExt())
                    .afterTime(0.5, arm.armDownSh())
                    .build());


            requestOpModeStop();
        }

    }

    public class Arm {
        private DcMotorEx Motor0SHLDR;
        private DcMotorEx Motor1EXT;
        private CRServo Servo0Wrist;
        private CRServo servo1;
        private CRServo servo2;
        private CRServo servo3;

        public Arm(HardwareMap hardwareMap) {
            Motor1EXT = hardwareMap.get(DcMotorEx.class, "Motor-1-EXT");
            Motor0SHLDR = hardwareMap.get(DcMotorEx.class, "Motor-0-SHLDR");
            Servo0Wrist = hardwareMap.get(CRServo.class, "Servo-0-Wrist");
            servo1 = hardwareMap.get(CRServo.class, "servo1");
            servo2 = hardwareMap.get(CRServo.class, "servo2");
            servo3 = hardwareMap.get(CRServo.class, "servo3");
            Motor1EXT.setDirection(DcMotorEx.Direction.FORWARD);
            Motor0SHLDR.setDirection(DcMotorEx.Direction.REVERSE);
            Servo0Wrist.setDirection(CRServo.Direction.FORWARD);
            servo1.setDirection(CRServo.Direction.REVERSE);
            servo2.setDirection(CRServo.Direction.FORWARD);
            servo3.setDirection(CRServo.Direction.FORWARD);
            Motor0SHLDR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Motor1EXT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Motor1EXT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor0SHLDR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor1EXT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Motor0SHLDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class ArmUpSh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor0SHLDR.setPower(1);
                    servo1.setPower(0.45);

                    Servo0Wrist.setPower(0.25);

                    initialized = true;
                }
                double position = Motor0SHLDR.getCurrentPosition();
                if (position < 2950) {
                    return true;
                } else {
                    Motor0SHLDR.setPower(0);
                    return false;
                }
            }
        }

        public Action armUpSh() {
            return new ArmUpSh();
        }

        public class ArmUpExt implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor1EXT.setPower(1);
                    Servo0Wrist.setPower(0.25);
                    servo1.setPower(0.45);
                    initialized = true;
                }
                double position = Motor1EXT.getCurrentPosition();
                if (position < 1450) {
                    return true;
                } else {
                    Motor1EXT.setPower(0);
                    return false;
                }
            }
        }

        public Action armUpExt() {
            return new ArmUpExt();
        }

        public class ArmDownSh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor0SHLDR.setPower(-1);
                    Servo0Wrist.setPower(0.05);
                    servo1.setPower(0);
                    initialized = true;
                }
                double position = Motor0SHLDR.getCurrentPosition();
                if (position > 10) {
                    return true;
                } else {
                    Motor0SHLDR.setPower(0);
                    return false;
                }
            }
        }

        public Action armDownSh() {
            return new ArmDownSh();
        }

        public class ArmDownExt implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor1EXT.setPower(-1);
                    Servo0Wrist.setPower(0.05);
                    servo1.setPower(0);
                    initialized = true;
                }
                double position = Motor1EXT.getCurrentPosition();
                if (position > 5) {
                    return true;
                } else {
                    Motor0SHLDR.setPower(0);
                    return false;
                }
            }
        }

        public Action armDownExt() {
            return new ArmDownExt();
        }


        public class ArmMidSh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor0SHLDR.setPower(-1);
                    Servo0Wrist.setPower(0.25);
                    servo1.setPower(0.45);
                    initialized = true;
                }
                double position = Motor0SHLDR.getCurrentPosition();
                if (position > 2600) {
                    return true;
                } else {
                    Motor0SHLDR.setPower(0);
                    servo1.setPower(0);
                    return false;
                }
            }
        }

        public Action armMidSh() {
            return new ArmMidSh();
        }


        public class OpenArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo1.setPower(0);
                return false;
            }
        }

        public Action openArm() {
            return new OpenArm();
        }

        public class CloseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo1.setPower(0.45);
                return false;
            }
        }

        public Action closeArm() {
            return new CloseArm();
        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo2.setPower(1);
                servo3.setPower(-1);
                sleep(4000);
                return false;
            }
        }

        public Action intake() {
            return new Intake();
        }

        public class IntakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo2.setPower(0);
                servo3.setPower(0);
                return false;
            }
        }

        public Action intakeStop() {
            return new IntakeStop();
        }
    }


    public class Claw {
        private DcMotorEx Motor3Lsh;
        private DcMotorEx Motor2Lext;
        private Servo subgrip;
        private Servo gripspin;

        public Claw(HardwareMap hardwareMap) {
            Motor2Lext = hardwareMap.get(DcMotorEx.class, "Motor-2-LEXT");
            Motor3Lsh = hardwareMap.get(DcMotorEx.class, "Motor-3-LSH");
            gripspin = hardwareMap.get(Servo.class, "GripSpin");
            subgrip = hardwareMap.get(Servo.class, "SubGrip");
            Motor2Lext.setDirection(DcMotorEx.Direction.REVERSE);
            Motor2Lext.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Motor3Lsh.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Motor2Lext.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor3Lsh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor2Lext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Motor3Lsh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class ClawUpSh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor3Lsh.setPower(0.4);
                    subgrip.setPosition(0.2);
                    initialized = true;
                }
                double position = Motor3Lsh.getCurrentPosition();
                if (position < 290) {
                    return true;
                } else {
                    Motor3Lsh.setPower(0);
                    return false;
                }
            }
        }

        public Action clawUpSh() {
            return new ClawUpSh();
        }

        public class ClawMid implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor2Lext.setPower(0.8);
                    subgrip.setPosition(0.2);
                    initialized = true;
                }
                double position = Motor2Lext.getCurrentPosition();
                if (position < 1880) {
                    return true;
                } else {
                    Motor2Lext.setPower(0);
                    return false;
                }
            }
        }

        public Action clawMid() {
            return new ClawMid();
        }

        public class ClawExt implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor2Lext.setPower(0.8);
                    subgrip.setPosition(0.2);
                    initialized = true;
                }
                double position = Motor2Lext.getCurrentPosition();
                if (position < 2250) {
                    return true;
                } else {
                    Motor2Lext.setPower(0);
                    return false;
                }
            }
        }

        public Action clawExt() {
            return new ClawExt();
        }

        public class ClawExtOrigin implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor2Lext.setPower(-0.8);
                    subgrip.setPosition(0.2);
                    initialized = true;
                }
                double position = Motor2Lext.getCurrentPosition();
                if (position > 15) {
                    return true;
                } else {
                    Motor2Lext.setPower(0);
                    return false;
                }
            }
        }

        public Action clawExtOrigin() {
            return new ClawExtOrigin();
        }

        public class ClawExtDownSh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor3Lsh.setPower(-0.4);
                    subgrip.setPosition(0.8);
                    initialized = true;
                }
                double position = Motor3Lsh.getCurrentPosition();
                if (position > 0) {
                    return true;
                } else {
                    Motor3Lsh.setPower(0);
                    subgrip.setPosition(0.2);
                    return false;
                }
            }
        }

        public Action clawExtDownSh() {
            return new ClawExtDownSh();
        }

        public class ClawMidDownSh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Motor3Lsh.setPower(-0.4);
                    subgrip.setPosition(0.8);
                    initialized = true;
                }
                double position = Motor3Lsh.getCurrentPosition();
                if (position > -250) {
                    return true;
                } else {
                    Motor3Lsh.setPower(0);
                    subgrip.setPosition(0.2);
                    return false;
                }
            }
        }

        public Action clawmidDownSh() {
            return new ClawMidDownSh();
        }


        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subgrip.setPosition(0.8);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                subgrip.setPosition(0.2);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }



    }



}