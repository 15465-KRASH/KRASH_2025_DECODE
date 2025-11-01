package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx flywheel; //Control Hub Motor Port 3
    public CRServo loader; //Servo Hub Servo Port 5
    public Servo hood; //Control Hub Servo Port 5
    PIDFCoefficients pidfCoefficients;

    private static final int ticksPerRev = 28;
    public int targetRPM = 2500;
    public int targetRPS = targetRPM / 60;
    public int targetSpeed = targetRPS * ticksPerRev;
    public int idleSpeed = 1000 / 60 / ticksPerRev;
    public double speedTol = 2 / 100.0; //Percent

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidfCoefficients =  flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidfCoefficients.i = 0;
        pidfCoefficients.d = 0.5;
        pidfCoefficients.f = 2;
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        loader = hardwareMap.get(CRServo.class, "loader");
        hood = hardwareMap.get(Servo.class, "hood");
    }

    public void shoot() {
        flywheel.setPower(0.5);
    }

    public void stop() {
        flywheel.setPower(0);
    }

    public void loadArtifact(double pwr) {
        loader.setPower(pwr);
    }

    public void spinUp() {
        flywheel.setVelocity(targetSpeed);
    }

    public void idle() {
        flywheel.setPower(0);
        //Uncomment below to keep idling instead of stopping
        //flywheel.setVelocity(idleSpeed);
    }

    public boolean atSpeed() {
        return Math.abs(flywheel.getVelocity() - targetSpeed) <= (targetSpeed * speedTol);
    }

    public void adjustHood() {
        //TODO: do math to adjust hood based off of Limelight distance to tag data
    }

    public void slowFeed() {
        loadArtifact(0.2);
    }

    public double getSpeed() {
        return flywheel.getVelocity();
    }
    public PIDFCoefficients showPIDFVals(){
        PIDFCoefficients pidfCoefficients = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P = ", pidfCoefficients.p);
        telemetry.addData("I = ", pidfCoefficients.i);
        telemetry.addData("D = ", pidfCoefficients.d);
        telemetry.addData("F = ", pidfCoefficients.f);
        return pidfCoefficients;
    }
    public void setPIDF(PIDFCoefficients pidf){
        flywheel.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    }

}
