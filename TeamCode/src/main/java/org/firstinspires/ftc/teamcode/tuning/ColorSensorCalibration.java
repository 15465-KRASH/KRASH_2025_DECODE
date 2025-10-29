package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.ButtonState;
import org.firstinspires.ftc.teamcode.classes.Spindexer;

@TeleOp(name = "Calibrate Color Sensors", group = "Calibration")
public class ColorSensorCalibration extends LinearOpMode {

    Spindexer spindexer;
    Spindexer.DetectedColor detectedColor;

    @Override
    public void runOpMode() throws InterruptedException {
        spindexer  = new Spindexer(hardwareMap, telemetry);

        ButtonState readIntake = new ButtonState(gamepad1, ButtonState.Button.a);
        ButtonState readLeft = new ButtonState(gamepad1, ButtonState.Button.x);
        ButtonState readRight = new ButtonState(gamepad1, ButtonState.Button.b);

        waitForStart();

        //spindexer.getDetectedColor(spindexer.intakeSensor, telemetry);
        //detectedColor = spindexer.getDetectedColor(spindexer.leftSensor, telemetry);
        //spindexer.getDetectedColor(spindexer.rightSensor, telemetry);

        while(opModeIsActive()) {
            telemetry.addData("color detected: ", detectedColor);

            if(readIntake.getCurrentPress()) {
                detectedColor = spindexer.getDetectedColor(spindexer.intakeSensor, telemetry);
                telemetry.addData("distance", spindexer.distanceIntakeSensor.getDistance(DistanceUnit.MM));
            }
            if(readLeft.getCurrentPress()) {
                detectedColor = spindexer.getDetectedColor(spindexer.leftSensor, telemetry);
            }
            if(readRight.getCurrentPress()) {
                detectedColor = spindexer.getDetectedColor(spindexer.rightSensor, telemetry);
            }

            telemetry.addData("Spindexer Pos", spindexer.getSpindexerPos());

            telemetry.update();
        }
    }
}
