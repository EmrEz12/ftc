package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.AutoCommandMachine;
import org.firstinspires.ftc.teamcode.lib.Command;
import org.firstinspires.ftc.teamcode.lib.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;
import org.firstinspires.ftc.teamcode.subsystems.ShoulderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "BasketAuto", preselectTeleOp = "Main")
public class BasketAuto extends LinearOpMode{
    private boolean busy = false;
    private Action hang1Action;
    private Action farSampleAction;
    private Action basket1Action;
    private Action centerSampleAction;
    private Action basket2Action;
    private Action wallSampleAction;
    private Action basket3Action;

    private MecanumDrive drive;

    private Command hang1Command = () -> followActionAsync(hang1Action);
    private Command farSampleCommand = () -> followActionAsync(farSampleAction);
    private Command basket1Command = () -> followActionAsync(basket1Action);
    private Command centerSampleCommand = () -> followActionAsync(centerSampleAction);
    private Command basket2Command = () -> followActionAsync(basket2Action);
    private Command wallSampleCommand = () -> followActionAsync(wallSampleAction);
    private Command basket3Command = () -> followActionAsync(basket3Action);

    private void followActionAsync(Action action){
        busy = true;
        Thread thread = new Thread(
                () -> {
                    Actions.runBlocking(action);
                    busy = false;
                }
        );
        thread.start();
    }

    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private ShoulderSubsystem shoulder;
    private WristSubsystem wrist;

    private Command busyTrue = () -> busy = true;
    private Command busyFalse = () -> busy = false;
    private Command wristUp = () -> wrist.rotateUp();
    private Command wristMid = () -> wrist.rotateMid();
    private Command wristOrigin = () -> wrist.rotateOrigin();
    private Command grabOpen = () -> wrist.HandOpen();
    private Command grabClose = () -> wrist.HandClose();
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

    private CommandSequence hang1Sequence = new CommandSequence()
            .addCommand(hang1Command)
            .addCommand(busyTrue)
            .addCommand(armHang)
            .build();
    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(farSampleCommand)
            .addCommand(busyTrue)
            .addCommand(busyFalse)
            .build();
    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(basket1Command)
            .addCommand(busyTrue)
            .addWaitCommand(0.2)
            .addCommand(busyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(centerSampleCommand)
            .addCommand(busyTrue)
            .addWaitCommand(2)
            .addCommand(busyFalse)
            .build();
    private CommandSequence basket2Sequence = new CommandSequence()
            .addCommand(basket2Command)
            .build();
    private CommandSequence wallSampleSequence = new CommandSequence()
            .addCommand(wallSampleCommand)
            .build();
    private CommandSequence basket3Sequence = new CommandSequence()
            .addCommand(basket3Command)
            .build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(hang1Sequence)
//            .addCommandSequence(farSampleSequence)
//            .addCommandSequence(basket1Sequence)
//            .addCommandSequence(centerSampleSequence)
//            .addCommandSequence(basket2Sequence)
//            .addCommandSequence(wallSampleSequence)
//            .addCommandSequence(basket3Sequence)
            .addCommandSequence(farSampleSequence)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        /*telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new ArmSubsystem(this);
        drive = new MecanumDrive(hardwareMap, 0);
        shoulder = new ShoulderSubsystem(this);
        intake = new IntakeSubsystem(this);
        wrist = new WristSubsystem(this);

        arm.init(hardwareMap);
        shoulder.init(hardwareMap);
        arm.ground();
        wrist.init(hardwareMap);
        intake.init(hardwareMap);*/


//        TrajectoryVelocityConstraint velConstraint = SampleMecanumDrive.getVelocityConstraint(20, 1, DriveConstants.TRACK_WIDTH);
//        TrajectoryAccelerationConstraint accelConstraint = SampleMecanumDrive.getAccelerationConstraint(30);
        /*
        hang1Action = drive
                .actionBuilder(BasketConstantsDash.START_POSE)
                .lineToY(basketConstants.CHAMBER.getVec().y)
                .setTangent(-Math.PI / 2)
                .build();
        farSampleAction = drive
                .actionBuilder(basketConstants.CHAMBER.getPose())
                .splineToLinearHeading(basketConstants.FAR_SAMPLE.getPose(), basketConstants.FAR_SAMPLE.getH())
                .setTangent(0)
                .build();
        basket1Action = drive
                .actionBuilder(basketConstants.FAR_SAMPLE.getPose())
                .splineToLinearHeading(basketConstants.BASKET_1.getPose(), basketConstants.BASKET_1.getH())
                .setTangent(Math.PI / 4)
                .build();
        centerSampleAction = drive
                .actionBuilder(basketConstants.BASKET_1.getPose())
                .splineToLinearHeading(basketConstants.CENTER_SAMPLE.getPose(), basketConstants.CENTER_SAMPLE.getH())
                .setTangent(-Math.PI / 4)
                .build();
        basket2Action = drive
                .actionBuilder(basketConstants.CENTER_SAMPLE.getPose())
                .splineToLinearHeading(basketConstants.BASKET_2.getPose(), basketConstants.BASKET_2.getH())
                .setTangent(Math.PI / 4)
                .build();
        wallSampleAction = drive
                .actionBuilder(basketConstants.BASKET_2.getPose())
                .splineToLinearHeading(basketConstants.WALL_SAMPLE.getPose(), basketConstants.WALL_SAMPLE.getH())
                .setTangent(-Math.PI / 4)
                .build();
        basket3Action = drive
                .actionBuilder(basketConstants.WALL_SAMPLE.getPose())
                .splineToLinearHeading(basketConstants.BASKET_3.getPose(), basketConstants.BASKET_3.getH())
                .build();

        while(opModeInInit() && !isStopRequested()){
            drive.updatePoseEstimate();
            telemetry.addData("drive x", drive.localizer.getPose().position.x);
            telemetry.addData("drive y", drive.localizer.getPose().position.y);
            telemetry.update();

        }

//        drive.localizer.setPose(BasketConstantsDash.START_POSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            drive.updatePoseEstimate();
            commandMachine.run(busy);
            telemetry.addData("drive x", drive.localizer.getPose().position.x);
            telemetry.addData("drive y", drive.localizer.getPose().position.y);
            telemetry.update();
        }

        //Thread.sleep(500);*/
    }
}