package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public CRServo leftLowRoller, rightLowRoller;
    public CRServo leftMidRoller, rightMidRoller;
    public CRServo highRoller;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        
    }
}