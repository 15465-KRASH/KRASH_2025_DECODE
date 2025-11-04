package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Lift;
import org.firstinspires.ftc.teamcode.classes.Shooter;
import org.firstinspires.ftc.teamcode.classes.Spindexer;
import org.firstinspires.ftc.teamcode.classes.vision.Vision;

import java.util.List;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public MecanumDrive drive;
    public Lift lift;
    public Shooter shooter;
    public Intake intake;
    public Spindexer spindexer;
    public Limelight3A limelight;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new MecanumDrive(hardwareMap, pose);
        lift = new Lift(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        spindexer = new Spindexer(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public double getAprilTagOffset () {
        LLResult llResult;
        llResult = limelight.getLatestResult();
        if (llResult != null) {
            if (llResult.isValid()) {
                // Access AprilTag results
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                        return fr.getTargetXDegrees();
                    }
                }
            }
        }
        return 0;
    }

    public double getTurnToTargetRotation() {
        LLResult llResult;
        llResult = limelight.getLatestResult();
        if (llResult != null) {
            if (llResult.isValid()) {
                // Access AprilTag results
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                        return fr.getTargetXDegrees();
                    }
                }
            }
        }
        return 0;
    }


    public Vector2d rotatedVector(Vector2d myVector, double angleDegrees){
        // For reference, this is what rotateBy() does internally
        double angleRadians = Math.toRadians(angleDegrees);
        double x_rotated = myVector.x * Math.cos(angleRadians) - myVector.y * Math.sin(angleRadians);
        double y_rotated = myVector.x * Math.sin(angleRadians) + myVector.y * Math.cos(angleRadians);
        return new Vector2d(x_rotated, y_rotated);
    }
}
