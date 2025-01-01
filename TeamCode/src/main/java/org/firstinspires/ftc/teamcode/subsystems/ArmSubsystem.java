package org.firstinspires.ftc.teamcode.subsystems;
import static com.sun.tools.javac.main.Option.S;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;


public class ArmSubsystem extends SubsystemBase {
    private DcMotor Motor22ndEXT;
    private DcMotor Motor1EXT;
    private DcMotor Motor0SHLDR;
    private CRServo Servo0WRIST;
    private CRServo servo1;
    private CRServo servo2;
    private CRServo servo3;
    private PIDController intakePID;
    private final double P = 1.0; //
    private final double I = 0.0; // Will need adjusting
    private final double D = 0.0; //


    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        Motor22ndEXT = hardwareMap.get(DcMotor.class, "Motor-2-2ndEXT");
        Motor1EXT = hardwareMap.get(DcMotor.class, "Motor-1-EXT");
        Motor0SHLDR = hardwareMap.get(DcMotor.class, "Motor-0-SHLDR");
        Servo0WRIST = hardwareMap.get(CRServo.class, "Servo-0-WRIST");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");

        intakePID = new PIDController(P, I, D);
    }

    public void intake(){
        servo2.setPower(1);
        servo3.setPower(-1);
    }

    public void eject(){
        servo2.setPower(-1);
        servo3.setPower(1);
    }

    public void intakestop(){
        servo2.setPower(0);
        servo3.setPower(0);
    }
    public void open(){
        servo1.setPower(-0.5);
    }
    public void close(){
        servo1.setPower(0.05);
    }
    public void wristUp(){
        Servo0WRIST.setPower(0.8);
    }
    public void wristmid(){
        Servo0WRIST.setPower(0.4);
    }
    public void wristOrigin(){
        Servo0WRIST.setPower(0);
    }



    public void ArmMove(double armPow, int armTrack) {
        if(armTrack == 1) {
            Motor0SHLDR.setPower(armPow * 1);
        }else if (armTrack == 2) {
            Motor1EXT.setPower(armPow * 1);
            Motor22ndEXT.setPower(armPow * -1);
        }
    }
  /*else if (slideTrack == 2) {
        motor_pivot.set(slidePow * ModifyingConstants.PIVOT_COEFFICIENT);
    }*/

    public double lengthToRotations(double angle, double gearRatio) {
        return (angle*gearRatio); // Calculated for a gear/wheel of radius 27.5 mm
    }
    //return Motor0SHLDR.getCurrentPosition();
    //return (motorNum == 1) ?motor_slideInner.getEncoder().getPosition() :motor_slideOuter.getEncoder().getPosition();

    public double getMotorRotation(int motorNum) {
        return (motorNum == 1) ?Motor0SHLDR.getCurrentPosition() :Motor1EXT.getCurrentPosition();
    }

    public double pidCalc(double err) {
        return intakePID.calculate(err);
    }


}
