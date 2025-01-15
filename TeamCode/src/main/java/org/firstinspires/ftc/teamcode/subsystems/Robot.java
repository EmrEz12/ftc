package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Robot extends Mechanism {
    public Scoring scoring = new Scoring(opMode);

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        scoring.init(hwMap);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        scoring.telemetry(telemetry);
    }

    @Override
    public void loop(Gamepad gamepad) {
        scoring.loop(gamepad);
    }
}