package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    public int maxSpindexerRot = 5;
    public int rot = 3*spindexerStep;
    public int spindexerTol = 10;

    public double intakeDistLimit = 75;

    public int[] intakeSpindexPos = {0, spindexerStep, -spindexerStep};
    public int[] shooterSpindexPos = {(int)Math.round(1.5*spindexerStep), (int)Math.round(-0.5*spindexerStep), (int)Math.round(0.5*spindexerStep)};

    public enum DetectedColor {
        GREEN,
        PURPLE,
        NONE
    }

    public DetectedColor[] spindexerSlots = {DetectedColor.NONE, DetectedColor.NONE,DetectedColor.NONE};

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

        clearAllSlots();
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

    public DetectedColor getIntakeColor(){
        return getDetectedColor(intakeSensor, telemetry);
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

    public void initSpindexerforAuton(){
        spindexerSlots[0] = DetectedColor.PURPLE;
        spindexerSlots[1] = DetectedColor.PURPLE;
        spindexerSlots[2] = DetectedColor.GREEN;
    }

    public DetectedColor getSlotColor(int x){
        return spindexerSlots[x];
    }

    public int getSpidexerPos() {
        return rotationMotor.getCurrentPosition();
    }

    public void runSpindexerPos(int pos){
        rotationMotor.setTargetPosition(pos);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(0.6);
    }

    public void stopSpindexer(){
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setPower(0);
    }

    public void setSlot(int slotNum, DetectedColor color){
        spindexerSlots[slotNum] = color;
    }

    public void clearSlot(int slotNum){
        setSlot(slotNum, DetectedColor.NONE);
    }

    public void clearAllSlots(){
        for(int x = 0; x <=2 ; x++){
            clearSlot(x);
        }
    }

    public void setSlotGreen(int slotNum){
        setSlot(slotNum, DetectedColor.GREEN);
    }

    public void setSlotPurple(int slotNum){
        setSlot(slotNum, DetectedColor.PURPLE);
    }

    public void moveToIntakePos(int slotnum){
        int target = intakeSpindexPos[slotnum];
        int finalTarget = calcNearestPos(target);
        runSpindexerPos(finalTarget);
    }

    public void moveToShooterPos(int slotnum){
        int target = shooterSpindexPos[slotnum];
        int finalTarget = calcNearestPos(target);
        runSpindexerPos(finalTarget);
    }

    public int calcNearestPos(int target) {
        int currentPos = getSpidexerPos();

        int currentRot = currentPos/rot;
        int targetopt1 = target + currentRot * rot;
        int targetopt2;
        if(targetopt1 < currentPos){
            targetopt2 = target + (currentRot+1) * rot;
        } else {
            targetopt2 = target + (currentRot-1) * rot;
        }

        int distance1 = Math.abs(targetopt1 - currentPos);
        int distance2 = Math.abs(targetopt2 - currentPos);

        int finalTarget;

        if((distance1 <= distance2 && Math.abs(targetopt1/rot) <= maxSpindexerRot) || distance1==0){
            finalTarget = targetopt1;
        } else if((distance2 <= distance1 && Math.abs(targetopt2/rot) <= maxSpindexerRot) || distance2==0) {
            finalTarget = targetopt2;
        } else if (Math.abs(targetopt1/rot) <= maxSpindexerRot){
            finalTarget = targetopt1;
        } else {
            finalTarget = targetopt2;
        }

//        System.out.println("CurrentPos = " + currentPos);
//        System.out.println("Rot = " + currentRot);
//        System.out.println("target = " + target);
//        System.out.println("targetopt1 = " + targetopt1);
//        System.out.println("targetopt2 = " + targetopt2);
//        System.out.println("distance1 = " + distance1);
//        System.out.println("distance2 = " + distance2);
//
//        System.out.println("finaltarget = " + finalTarget);

        return finalTarget;
    }

    public int findEmptyIntakeSlot(){
        int bestSlot = -1;
        int bestDistance = Integer.MAX_VALUE;
        int currentPos = getSpidexerPos();

        for (int i = 0; i <= 2; i++) {
            if(spindexerSlots[i] == DetectedColor.NONE){
                int target = intakeSpindexPos[i];
                int distance = Math.abs(calcNearestPos(target) - currentPos);
                if(distance < bestDistance){
                    bestSlot = i;
                }
            }
        }

        return bestSlot;
    }

    public int gotoClosestEmptyIntake(){
        int targetSlot = findEmptyIntakeSlot();
        if(targetSlot != -1){
            moveToIntakePos(targetSlot);
            return targetSlot;
        } else {
            return -1;
        }
    }

    public boolean isIntakeSlotFull(){
        return distanceIntakeSensor.getDistance(DistanceUnit.MM) < intakeDistLimit;
    }

    public boolean spindexerAtTarget(){
        return Math.abs(rotationMotor.getTargetPosition() - rotationMotor.getCurrentPosition()) < spindexerTol;
    }






}
