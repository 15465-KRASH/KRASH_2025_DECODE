package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    //leftClimbMotor - Expansion Hub Motor Port 2
    //rightClimbMotor - Control Hub Motor Port 2
    public DcMotorEx leftClimbMotor, rightClimbMotor;

    public static double KV = 1.0;
    public static double KP = 1.0;
    public static double KvP = 1.0;
    public static double KvD = 1.0;
    public static double KvI = 1.0;
    public static double KF = 1.0;

    public Lift (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        leftClimbMotor = hardwareMap.get(DcMotorEx.class, "leftClimbMotor");
        rightClimbMotor = hardwareMap.get(DcMotorEx.class, "rightClimbMotor");

        leftClimbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimbMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightClimbMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        setPIDFValues();

    }

    public void runLift(int pose) {
        leftClimbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightClimbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimbMotor.setTargetPosition(pose);
        rightClimbMotor.setTargetPosition(pose);
    }

    public void stopLift() {
        leftClimbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftClimbMotor.setPower(0);
        rightClimbMotor.setPower(0);
    }

    public void setPIDFValues() {
        leftClimbMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(KP, 0, 0, 0));
        rightClimbMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(KP, 0, 0, 0));

        leftClimbMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(KvP, KvI, KvD, KV));
        rightClimbMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(KvP, KvI, KvD, KV));
    }
}
