package org.firstinspires.ftc.teamcode.classes;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx flywheel; //Control Hub Motor Port 3
    public CRServo loader; //Servo Hub Servo Port 5
    public Servo hood; //Control Hub Servo Port 5
    PIDFCoefficients pidfCoefficients;

    //PIDEx Setup
    static double Kp = 0;
    static double Ki = 0;
    static double Kd = 0;
    static double Kv = 0; //1.1;
    static double Ka = 0; //0.2;
    static double Ks = 0; //0.001;
    static double targetAccelTime = 0.5; //seconds

    PIDCoefficientsEx pidExCoeff = new PIDCoefficientsEx(Kp, Ki, Kd, 0.9, 10, 1);
    PIDEx motorController = new PIDEx(pidExCoeff);

    FeedforwardCoefficients ffCoeff = new FeedforwardCoefficients(Kv,Ka,Ks);
    BasicFeedforward motorFFController = new BasicFeedforward(ffCoeff);

    public static final int ticksPerRev = 28;
    public int targetRPM = 3250;
    public int targetRPS = targetRPM / 60;
    public int targetSpeed = targetRPS * ticksPerRev;
    public int idleSpeed = 1000 / 60 / ticksPerRev;
    public double speedTol = 2 / 100.0; //Percent
    private static int reference = 1517; // targetSpeed (ticks/sec) for 3250 RPM

    public double hoodMin = 0;
    public double hoodMax = 0;

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
        loader.setDirection(DcMotorSimple.Direction.REVERSE);
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);

        hood.setPosition(0);
    }

    public void updateController(){
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentSpeed = flywheel.getVelocity();
        double targetAccel = (targetSpeed - currentSpeed) / targetAccelTime;
        double pidOutput = motorController.calculate(targetSpeed, currentSpeed);
        double ffOutput = motorFFController.calculate(0, targetSpeed, targetAccel);
        flywheel.setPower(Range.clip(pidOutput + ffOutput, -1.0, 1.0));
    }
    public Action updateFlywheel() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                updateController();
                return true;
            }
        };
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

    public void setTargetSpeed(int rpm){
        targetRPM = rpm;
        targetRPS = targetRPM / 60;
        targetSpeed = targetRPS * ticksPerRev;
    }

    public void spinUp(int target) {
        setTargetSpeed(target);
    }

    public void idle() {
        targetRPM = 0;
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

    public void setHood(double pos){
        hood.setPosition(pos);
    }

    public void slowFeed() {
        loadArtifact(0.2);
    }

    public double getSpeed() {
        return flywheel.getVelocity();
    }
    public PIDFCoefficients showPIDFVals(){
        PIDFCoefficients pidfCoefficients = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("P = ", pidfCoefficients.p);
//        telemetry.addData("I = ", pidfCoefficients.i);
//        telemetry.addData("D = ", pidfCoefficients.d);
//        telemetry.addData("F = ", pidfCoefficients.f);
        return pidfCoefficients;
    }
    public void setPIDF(PIDFCoefficients pidf){
        flywheel.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    }

    public void setPIDFExCoeeficients(PIDCoefficientsEx pidExCoeff, FeedforwardCoefficients ffCoeff){
        motorController = new PIDEx(pidExCoeff);
        motorFFController = new BasicFeedforward(ffCoeff);
    }

}
