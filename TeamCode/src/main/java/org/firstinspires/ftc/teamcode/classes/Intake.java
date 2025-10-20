package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public CRServo leftLowRoller, rightLowRoller;
    public CRServo leftMidRoller, rightMidRoller;
    public CRServo highRoller;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        leftLowRoller = hardwareMap.get(CRServo.class, "leftLowRoller");
        rightLowRoller = hardwareMap.get(CRServo.class, "rightLowRoller");

        leftMidRoller = hardwareMap.get(CRServo.class, "leftMidRoller");
        rightMidRoller = hardwareMap.get(CRServo.class, "rightMidRoller");

        highRoller = hardwareMap.get(CRServo.class, "highRoller");
    }

    public void intakeArtifact() {
        leftLowRoller.setPower(1);
        rightLowRoller.setPower(1);

        leftMidRoller.setPower(1);
        rightMidRoller.setPower(1);

        highRoller.setPower(1);
    }

    public void spitArtifacts() {
        leftLowRoller.setPower(-1);
        rightLowRoller.setPower(-1);

        leftMidRoller.setPower(-1);
        rightMidRoller.setPower(-1);

        highRoller.setPower(-1);
    }

    public void stop() {
        leftLowRoller.setPower(0);
        rightLowRoller.setPower(0);

        leftMidRoller.setPower(0);
        rightMidRoller.setPower(0);

        highRoller.setPower(0);
    }

    public void autoIntake() {

    }
}