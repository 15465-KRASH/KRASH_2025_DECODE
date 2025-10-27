package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx flywheel; //Control Hub Motor Port 3
    public CRServo loader; //Servo Hub Servo Port 5
    public Servo hood; //Control Hub Servo Port 5

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        loader = hardwareMap.get(CRServo.class, "loaderServo");
        hood = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void shoot() {
        flywheel.setPower(1);
    }

    public void loadArtifact() {
        loader.setPower(1);
    }

    public void adjustHood() {
        //TODO: do math to adjust hood based off of Limelight distance to tag data
    }
}
