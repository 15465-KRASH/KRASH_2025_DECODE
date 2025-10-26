package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindexer {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Spindexer spindexer;

    public DcMotorEx rotationMotor;
    public NormalizedColorSensor intakeSensor, leftSensor, rightSensor;

    boolean pickedUp = false;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        NONE
    }

    public DetectedColor getDetectedColor(NormalizedColorSensor sensor, Telemetry telemetry) {
        //return red, green, blue, alpha (amount of light)
        NormalizedRGBA colors = sensor.getNormalizedColors();

        //take light out of the equation, making the color output consistent no matter the lighting
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO: add if statements for the specific colors
        /*
        COLOR = red, green, blue (use rough inequalities, not exact values)
        PURPLE =
        GREEN =
         */

        return DetectedColor.NONE;
    }

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //assign values based on what's configured on the robot
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        leftSensor = hardwareMap.get(NormalizedColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(NormalizedColorSensor.class, "rightSensor");

        rotationMotor = hardwareMap.get(DcMotorEx.class, "spindexerRotationMotor");

        //TODO: play around with the gain of the sensors during calibration (.setGain(int))
    }

    public void rotate(int pose) {
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setTargetPosition(pose);
        rotationMotor.setPower(1);
    }

    public void stop() {
        rotationMotor.setPower(0);
    }

    public void rotateAfterPickup() {
        if (spindexer.getDetectedColor(intakeSensor, telemetry) == DetectedColor.PURPLE || spindexer.getDetectedColor(intakeSensor, telemetry) == DetectedColor.GREEN) {
            spindexer.rotate(60);
            pickedUp = true;
        }
        if (pickedUp == true && spindexer.getDetectedColor(intakeSensor, telemetry) == DetectedColor.PURPLE || spindexer.getDetectedColor(intakeSensor, telemetry) == DetectedColor.GREEN) {
            spindexer.stop();
            pickedUp = false;
        } else if (pickedUp == true && spindexer.getDetectedColor(intakeSensor, telemetry) == DetectedColor.NONE) {
            spindexer.stop();
            pickedUp = false;
        }
    }

}
