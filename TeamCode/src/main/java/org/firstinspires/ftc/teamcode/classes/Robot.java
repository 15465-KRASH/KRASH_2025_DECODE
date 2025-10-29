package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.vision.Vision;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public Intake intake;
    public Spindexer spindexer;
    public Shooter shooter;
    public Lift lift;
    public Vision vision;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = new Intake(hardwareMap, telemetry);
        spindexer = new Spindexer(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        vision = Vision.createDefault();
    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
