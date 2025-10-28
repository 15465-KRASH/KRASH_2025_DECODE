package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindexer {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Spindexer spindexer;

    public DcMotorEx rotationMotor; //Expansion Hub Motor Port 3
    /*
    * intakeSensor - Control Hub I2C Port 3
    * leftSensor - Expansion Hub I2C Port 3
    * rightSensor - Control Hub I2C Port 1
    * */
    public NormalizedColorSensor intakeSensor, leftSensor, rightSensor;
    public DistanceSensor distanceIntakeSensor;

    boolean pickedUp = false;

    public int spindexerStep = 179;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        NONE
    }

    public DetectedColor[] spindexerPositions = {DetectedColor.NONE, DetectedColor.NONE,DetectedColor.NONE};

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

        /*
        COLOR = red, green, blue (use rough inequalities, not exact values)
        these are for NOT the intake sensor
        PURPLE = r<0.9, g<1.1, b>1.1
        GREEN = r<0.5, g>1.1, b>0.9
         */

        /*
        COLOR = red, green, blue (use rough inequalities, not exact values)
        these are FOR the intake sensor
        PURPLE = r>1.1, g<2.0, b>1.9
        GREEN = r<1.1, g>2.1, b>1.8
         */
        if (sensor != intakeSensor) {
            if (normRed < 0.9 && normGreen < 1.1 && normBlue > 1.1) {
                return DetectedColor.PURPLE;
            } else if (normRed < 0.5 && normGreen > 1.1 && normBlue > 0.9) {
                return DetectedColor.GREEN;
            } else {
                return DetectedColor.NONE;
            }
        } else if (sensor == intakeSensor) {
            if (normRed < 1.8 && normGreen < 2.0 && normBlue > 1.9) {
                return DetectedColor.PURPLE;
            } else if (normRed < 1.1 && normGreen > 2.1 && normBlue > 1.8) {
                return DetectedColor.GREEN;
            } else {
                return DetectedColor.NONE;
            }
        } else {
            return DetectedColor.NONE;
        }

    }

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //assign values based on what's configured on the robot
        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        distanceIntakeSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");

        leftSensor = hardwareMap.get(NormalizedColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(NormalizedColorSensor.class, "rightSensor");

        rotationMotor = hardwareMap.get(DcMotorEx.class, "spindexerRotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeSensor.setGain(100);
        leftSensor.setGain(100);
        rightSensor.setGain(100);
    }

    public void rotate(int pose) {
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setTargetPosition(pose);
        //rotationMotor.setPower(1);
    }

    public void stop() {
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void initSpindexerArray(){
        spindexerPositions[0] = DetectedColor.GREEN;
        spindexerPositions[1] = DetectedColor.PURPLE;
        spindexerPositions[2] = DetectedColor.PURPLE;
    }

    public int getSpidexerPos() {
        return rotationMotor.getCurrentPosition();
    }

    public void runSpindexerPos(int pos){
        rotationMotor.setTargetPosition(pos);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(0.3);
    }

    public void stopSpindexer(){
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setPower(0);
    }

}
