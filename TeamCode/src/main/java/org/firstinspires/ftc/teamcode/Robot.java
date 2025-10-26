package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public MecanumDrive drive;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new MecanumDrive(hardwareMap, pose);

    }

}
