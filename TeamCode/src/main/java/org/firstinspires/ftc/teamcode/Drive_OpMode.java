/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.classes.Robot;

import org.firstinspires.ftc.teamcode.classes.ButtonState;
import org.firstinspires.ftc.teamcode.classes.vision.Vision;

import java.util.ArrayList;
import java.util.List;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive", group="Comp")
//@Disabled
public class Drive_OpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        TelemetryPacket packet = new TelemetryPacket();
        Robot m_robot = new Robot(hardwareMap, telemetry, new Pose2d(0,0,0));

        ButtonState liftTester =  new ButtonState(gamepad1, ButtonState.Button.a);
        ButtonState loaderTest = new ButtonState(gamepad1, ButtonState.Button.b);

//        ButtonState highRollerTest = new ButtonState(gamepad2, ButtonState.Button.y);
//        ButtonState leftMidRollerTest = new ButtonState(gamepad2, ButtonState.Button.x);
//        ButtonState rightMidRollerTest = new ButtonState(gamepad2, ButtonState.Button.b);
//        ButtonState leftLowRollerTest = new ButtonState(gamepad2, ButtonState.Button.dpad_left);
//        ButtonState rightLowRollerTest = new ButtonState(gamepad2, ButtonState.Button.dpad_right);

        ButtonState intakeArtifact = new ButtonState(gamepad2, ButtonState.Button.x);
        ButtonState spitArtifact = new ButtonState(gamepad2, ButtonState.Button.b);
        ButtonState stopIntake = new ButtonState(gamepad2, ButtonState.Button.a);

        ButtonState testSpin = new ButtonState(gamepad2, ButtonState.Button.dpad_up);
        ButtonState spinTen = new ButtonState(gamepad2, ButtonState.Button.dpad_down);

        ButtonState driveToTag = new ButtonState(gamepad1, ButtonState.Button.a);

        double powerScale=1;

        PoseVelocity2d driveControl;

        /**************************
        Example button state setups
         **************************/
//        ButtonState autoIntake = new ButtonState(gamepad2, ButtonState.Button.right_trigger);
//        ButtonState autoBackIntake = new ButtonState(gamepad2, ButtonState.Button.right_bumper);
//        ButtonState zeroLift = new ButtonState(gamepad1, ButtonState.Button.back);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("Extension Position", m_robot.intake.getCurrentExt());
//            telemetry.addData("Lift Position", m_robot.lift.getCurrentExt());

            Vision.TargetData td = m_robot.vision.processFrame();
            telemetry.addData("Valid Target: ", td.isAcquired);
            if (td.isAcquired && td.botPose != null) {
                telemetry.addData("X", td.xPosition);
                telemetry.addData("Y", td.yPosition);
                telemetry.addData("Z", td.zPosition);
            }

            if (gamepad1.right_bumper) {
                powerScale=1;
            } else if (gamepad1.left_bumper) {
                powerScale=.5;
            }

            Vector2d input = new Vector2d(
                    Math.pow(-gamepad1.left_stick_y, 3) * powerScale,
                    Math.pow(-gamepad1.left_stick_x, 3) * powerScale);

            double rotation = Math.pow(-gamepad1.right_stick_x, 3) * powerScale;

            driveControl = new PoseVelocity2d(input, rotation);

            m_robot.drive.setDrivePowers(driveControl);

            if (driveToTag.buttonHeld) {

            }

            if (liftTester.getCurrentPress()) {
                m_robot.lift.manualClimb(0.2);
            } else {
                m_robot.lift.stopLift();
            }

            if (loaderTest.getCurrentPress()) {
                m_robot.shooter.loadArtifact(1.0);
            }

            if(testSpin.newPress()){
                int currentPos = m_robot.spindexer.getSpidexerPos();
                int newPos = currentPos - m_robot.spindexer.spindexerStep * 30;
                m_robot.spindexer.runSpindexerPos(newPos);
            }

            if(spinTen.newPress()){
                int currentPos = m_robot.spindexer.getSpidexerPos();
                int newPos = currentPos + m_robot.spindexer.spindexerStep * 30;
                m_robot.spindexer.runSpindexerPos(newPos);
            }



            /*******************************
             * Servo Direction Test
             *******************************/
//            if(highRollerTest.getCurrentPress()){
//                m_robot.intake.highRoller.setPower(1);
//            } else {
//                m_robot.intake.highRoller.setPower(0);
//            }
//            if(leftMidRollerTest.getCurrentPress()){
//                m_robot.intake.leftMidRoller.setPower(1);
//            } else {
//                m_robot.intake.leftMidRoller.setPower(0);
//            }
//            if(rightMidRollerTest.getCurrentPress()){
//                m_robot.intake.rightMidRoller.setPower(1);
//            } else {
//                m_robot.intake.rightMidRoller.setPower(0);
//            }
//            if(leftLowRollerTest.getCurrentPress()){
//                m_robot.intake.leftLowRoller.setPower(1);
//            } else {
//                m_robot.intake.leftLowRoller.setPower(0);
//            }
//            if(rightLowRollerTest.getCurrentPress()){
//                m_robot.intake.rightLowRoller.setPower(1);
//            } else {
//                m_robot.intake.rightLowRoller.setPower(0);
//            }

            if(intakeArtifact.getCurrentPress()){
                m_robot.intake.intakeArtifact();
            }
            if(spitArtifact.getCurrentPress()){
                m_robot.intake.spitArtifacts();
            }
            if(stopIntake.getCurrentPress()){
                m_robot.intake.stop();
            }

            telemetry.addData("Spindexer Pos", m_robot.spindexer.getSpidexerPos());
            telemetry.update();

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
