package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx flywheel;
    public CRServo loader;
    public Servo hood;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        loader = hardwareMap.get(CRServo.class, "loader");
        hood = hardwareMap.get(Servo.class, "hood");
    }

    public void shoot() {
        flywheel.setPower(1);
    }

    public void loadArtifact(double pwr) {
        loader.setPower(pwr);
    }

    public void adjustHood() {
        //TODO: do math to adjust hood based off of Limelight distance to tag data
    }
}
