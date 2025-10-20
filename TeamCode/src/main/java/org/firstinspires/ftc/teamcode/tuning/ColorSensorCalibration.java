package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.Spindexer;

@TeleOp(name = "Calibrate Color Sensors", group = "Calibration")
public class ColorSensorCalibration extends OpMode {

    Spindexer spindexer;
    Spindexer.DetectedColor detectedColor;

    @Override
    public void init() {
        spindexer  = new Spindexer(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        spindexer.getDetectedColor(spindexer.intakeSensor, telemetry);
        spindexer.getDetectedColor(spindexer.leftSensor, telemetry);
        spindexer.getDetectedColor(spindexer.rightSensor, telemetry);

        telemetry.addData("color detected: ", detectedColor);
    }
}
