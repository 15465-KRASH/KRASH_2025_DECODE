package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.Lift;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public MecanumDrive drive;
    public Lift lift;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new MecanumDrive(hardwareMap, pose);
        lift = new Lift(hardwareMap, telemetry);

    }

}
