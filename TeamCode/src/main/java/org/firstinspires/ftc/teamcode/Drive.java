package org.firstinspires.ftc.teamcode;

import android.app.Notification;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.ButtonState;
import org.firstinspires.ftc.teamcode.classes.HeadingStorage;
import org.firstinspires.ftc.teamcode.classes.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Drive", group="Comp")
public class Drive extends LinearOpMode {
    Robot m_robot;

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        TelemetryPacket packet = new TelemetryPacket();
        m_robot = new Robot(hardwareMap, telemetry, HeadingStorage.startingPose);

        ButtonState liftTest = new ButtonState(gamepad2, ButtonState.Button.b);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        //run until end of match (driver presses STOP)
        while (opModeIsActive()) {

            if (liftTest.newPress()) {
                m_robot.lift.runLift(4);
            }

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
