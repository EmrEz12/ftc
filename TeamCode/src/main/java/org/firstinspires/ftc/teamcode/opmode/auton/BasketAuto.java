package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.opmode.auton.BasketConstantsDash.basketConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.geometry.Vector2d;
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
    private BasketConstants basketConstants = BasketConstantsDash.basketConstants;
    private Action hang1Action;
    private Action farSampleAction;
    private Action basket1Action;
    private Action centerSampleAction;
    private Action basket2Action;
    private Action wallSampleAction;
    private Action basket3Action;
    private Action gop4Action;
    private Action push2Action;
    private Action gop5Action;
    private Action gop6Action;
    private Action push3Action;
    private Action get1Action;
    private Action put1Action;
    private Action get2Action;
    private Action put3Action;
    private Action get4Action;
    private Action put4Action;
    private Action parkAction;
    private Action get3Action;
    private Action put2Action;

    private MecanumDrive drive;

    private Command hang1Command = () -> followActionAsync(hang1Action);
    private Command farSampleCommand = () -> followActionAsync(farSampleAction);
    private Command basket1Command = () -> followActionAsync(basket1Action);
    private Command centerSampleCommand = () -> followActionAsync(centerSampleAction);
    private Command basket2Command = () -> followActionAsync(basket2Action);
    private Command wallSampleCommand = () -> followActionAsync(wallSampleAction);
    private Command basket3Command = () -> followActionAsync(basket3Action);
    private Command gop4Command = () -> followActionAsync(gop4Action);
    private Command push2Command = () -> followActionAsync(push2Action);
    private Command gop5Command = () -> followActionAsync(gop5Action);
    private Command gop6Command = () -> followActionAsync(gop6Action);
    private Command push3Command = () -> followActionAsync(push3Action);
    private Command get1Command = () -> followActionAsync(get1Action);
    private Command put1Command = () -> followActionAsync(put1Action);
    private Command get2Command = () -> followActionAsync(get2Action);
    private Command put2Command = () -> followActionAsync(put3Action);
    private Command get3Command = () -> followActionAsync(get2Action);
    private Command put3Command = () -> followActionAsync(put3Action);
    private Command get4Command = () -> followActionAsync(get4Action);
    private Command put4Command = () -> followActionAsync(put4Action);
    private Command parkCommand = () -> followActionAsync(parkAction);

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
    private Command shHangdown = () -> shoulder.shhangdown();
    private Command shopenintake = () -> shoulder.shopenintake();
    private Command armFront = () -> arm.intextended();
    private Command armWall = () -> arm.wall();
    private Command armDownIntake = () -> arm.ground();
    private Command armHang = () -> arm.hang();
    private Command armBucket = () -> arm.bucket();

    private CommandSequence hang1Sequence = new CommandSequence()
            .addCommand(shHang)
            //.addCommand(busyTrue)
            .addCommand(armHang)
            .addCommand(wristMid)
            .addCommand(hang1Command)
            .build();
    private CommandSequence farSampleSequence = new CommandSequence()
            .addCommand(armWall)
            .addCommand(shWall)
            .addCommand(wristMid)
            .addCommand(farSampleCommand)
            //.addCommand(busyTrue)
            //.addCommand(busyFalse)
            .build();
    private CommandSequence basket1Sequence = new CommandSequence()
            .addCommand(basket1Command)
            //.addCommand(busyTrue)
            //.addWaitCommand(0.2)
            //.addCommand(busyFalse)
            .build();
    private CommandSequence centerSampleSequence = new CommandSequence()
            .addCommand(centerSampleCommand)
            //.addCommand(busyTrue)
            //.addWaitCommand(2)
            //.addCommand(busyFalse)
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
    private CommandSequence push2Sequence = new CommandSequence()
            .addCommand(push2Command)
            .build();
    private CommandSequence gop5Sequence = new CommandSequence()
            .addCommand(gop5Command)
            .build();
    private CommandSequence gop6Sequence = new CommandSequence()
            .addCommand(gop6Command)
            .build();
    private CommandSequence push3Sequence = new CommandSequence()
            .addCommand(push3Command)
            .addCommand(grabOpen)
            .build();
    private CommandSequence get1Sequence = new CommandSequence()
            .addCommand(get1Command)
            .addCommand(grabClose)
            .build();
    private CommandSequence put1Sequence = new CommandSequence()
            .addCommand(put1Command)
            .addCommand(shHang)
            .addCommand(armHang)
            .addCommand(wristMid)
            .addCommand(shHangdown)
            .addCommand(grabOpen)
            .build();
    private CommandSequence get2Sequence = new CommandSequence()
            .addCommand(get2Command)
            .addCommand(grabClose)
            .build();
    private CommandSequence put2Sequence = new CommandSequence()
            .addCommand(put2Command)
            .addCommand(shHang)
            .addCommand(armHang)
            .addCommand(wristMid)
            .addCommand(shHangdown)
            .addCommand(grabOpen)
            .build();
    private CommandSequence get3Sequence = new CommandSequence()
            .addCommand(get3Command)
            .addCommand(grabClose)
            .build();
    private CommandSequence put3Sequence = new CommandSequence()
            .addCommand(put3Command)
            .addCommand(shHang)
            .addCommand(armHang)
            .addCommand(wristMid)
            .addCommand(shHangdown)
            .addCommand(grabOpen)
            .build();
    private CommandSequence get4Sequence = new CommandSequence()
            .addCommand(get4Command)
            .addCommand(grabClose)
            .build();
    private CommandSequence put4Sequence = new CommandSequence()
            .addCommand(put4Command)
            .addCommand(shHang)
            .addCommand(armHang)
            .addCommand(wristMid)
            .addCommand(shHangdown)
            .addCommand(grabOpen)
            .build();
    private CommandSequence parkSequence = new CommandSequence()
            .addCommand(parkCommand)
            .build();

    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(hang1Sequence)
            .addCommandSequence(farSampleSequence)
            .addCommandSequence(basket1Sequence)
            .addCommandSequence(centerSampleSequence)
            .addCommandSequence(basket2Sequence)
            .addCommandSequence(wallSampleSequence)
            .addCommandSequence(basket3Sequence)
            .addCommandSequence(push2Sequence)
            .addCommandSequence(gop5Sequence)
            .addCommandSequence(gop6Sequence)
            .addCommandSequence(push3Sequence)
            .addCommandSequence(get1Sequence)
            .addCommandSequence(put1Sequence)
            .addCommandSequence(get2Sequence)
            .addCommandSequence(put2Sequence)
            .addCommandSequence(get3Sequence)
            .addCommandSequence(put3Sequence)
            .addCommandSequence(get4Sequence)
            .addCommandSequence(put4Sequence)
            .addCommandSequence(parkSequence)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new ArmSubsystem(this);
        drive = new MecanumDrive(hardwareMap, BasketConstantsDash.START_POSE);
        shoulder = new ShoulderSubsystem(this);
        intake = new IntakeSubsystem(this);
        wrist = new WristSubsystem(this);

        arm.init(hardwareMap);
        shoulder.init(hardwareMap);
        arm.ground();
        wrist.init(hardwareMap);
        intake.init(hardwareMap);


//        TrajectoryVelocityConstraint velConstraint = SampleMecanumDrive.getVelocityConstraint(20, 1, DriveConstants.TRACK_WIDTH);
//        TrajectoryAccelerationConstraint accelConstraint = SampleMecanumDrive.getAccelerationConstraint(30);

        hang1Action = drive
                .actionBuilder(BasketConstantsDash.START_POSE)
                .splineToLinearHeading(basketConstants.FORWARD.getPose(), basketConstants.FORWARD.getH())
                .build();
        farSampleAction = drive
                .actionBuilder(basketConstants.FORWARD.getPose())
                .splineToLinearHeading(basketConstants.Go1.getPose(), basketConstants.Go1.getH())
                .build();
        basket1Action = drive
                .actionBuilder(basketConstants.Go1.getPose())
                .splineToLinearHeading(basketConstants.Gop1.getPose(), basketConstants.Gop1.getH())
                .build();
        centerSampleAction = drive
                .actionBuilder(basketConstants.Gop1.getPose())
                .splineToLinearHeading(basketConstants.Gop2.getPose(), basketConstants.Gop2.getH())
                .build();
        basket2Action = drive
                .actionBuilder(basketConstants.Gop2.getPose())
                .splineToLinearHeading(basketConstants.Push1.getPose(), basketConstants.Push1.getH())
                .build();
        wallSampleAction = drive
                .actionBuilder(basketConstants.Push1.getPose())
                .splineToLinearHeading(basketConstants.Gop3.getPose(), basketConstants.Gop4.getH())
                .build();
        basket3Action = drive
                .actionBuilder(basketConstants.Gop3.getPose())
                .splineToLinearHeading(basketConstants.Gop4.getPose(), basketConstants.Gop4.getH())
                .build();
        push2Action = drive
                .actionBuilder(basketConstants.Gop4.getPose())
                .splineToLinearHeading(basketConstants.Push2.getPose(), basketConstants.Push2.getH())
                .build();
        gop5Action = drive
                .actionBuilder(basketConstants.Push2.getPose())
                .splineToLinearHeading(basketConstants.Gop5.getPose(), basketConstants.Gop5.getH())
                .build();
        gop6Action = drive
                .actionBuilder(basketConstants.Gop5.getPose())
                .splineToLinearHeading(basketConstants.Gop6.getPose(), basketConstants.Gop6.getH())
                .build();
        push3Action = drive
                .actionBuilder(basketConstants.Gop6.getPose())
                .splineToLinearHeading(basketConstants.Push3.getPose(), basketConstants.Push3.getH())
                .build();
        get1Action = drive
                .actionBuilder(basketConstants.Push3.getPose())
                .splineToLinearHeading(basketConstants.Get1.getPose(), basketConstants.Get1.getH())
                .build();
        put1Action = drive
                .actionBuilder(basketConstants.Get1.getPose())
                .splineToLinearHeading(basketConstants.Put1.getPose(), basketConstants.Put1.getH())
                .build();
        get2Action = drive
                .actionBuilder(basketConstants.Put1.getPose())
                .splineToLinearHeading(basketConstants.Get2.getPose(), basketConstants.Get2.getH())
                .build();
        put2Action = drive
                .actionBuilder(basketConstants.Get2.getPose())
                .splineToLinearHeading(basketConstants.Put2.getPose(), basketConstants.Put2.getH())
                .build();
        get3Action = drive
                .actionBuilder(basketConstants.Put2.getPose())
                .splineToLinearHeading(basketConstants.Get3.getPose(), basketConstants.Get3.getH())
                .build();
        put3Action = drive
                .actionBuilder(basketConstants.Get3.getPose())
                .splineToLinearHeading(basketConstants.Put3.getPose(), basketConstants.Put3.getH())
                .build();
        get4Action = drive
                .actionBuilder(basketConstants.Put3.getPose())
                .splineToLinearHeading(basketConstants.Get4.getPose(), basketConstants.Get4.getH())
                .build();
        put4Action = drive
                .actionBuilder(basketConstants.Get4.getPose())
                .splineToLinearHeading(basketConstants.Put4.getPose(), basketConstants.Put4.getH())
                .build();
        parkAction = drive
                .actionBuilder(basketConstants.Put4.getPose())
                .splineToLinearHeading(basketConstants.Park.getPose(), basketConstants.Park.getH())
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