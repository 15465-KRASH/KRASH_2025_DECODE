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

import android.widget.Button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.actions.IntakeArtifact;
import org.firstinspires.ftc.teamcode.actions.ShootAll;
import org.firstinspires.ftc.teamcode.actions.ShootAllVariant;
import org.firstinspires.ftc.teamcode.classes.ButtonState;

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

    enum PIDFVals {
        P,
        I,
        D,
        F;

        private static final PIDFVals[] vals = values();

        public PIDFVals next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
        public PIDFVals prev() {
            return vals[(this.ordinal() - 1 + vals.length) % vals.length];
        }
    }



    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        TelemetryPacket packet = new TelemetryPacket();
        Robot m_robot = new Robot(hardwareMap, telemetry, new Pose2d(0,0,0));

        PIDFCoefficients pidf = m_robot.shooter.showPIDFVals();
        pidf.p = 22;
        pidf.i = 3;
        pidf.d = 0;
        pidf.f = 0;


        IntakeArtifact intakeAction = new IntakeArtifact(m_robot.intake, m_robot.spindexer);
        ShootAllVariant shootAction = new ShootAllVariant(m_robot.shooter, m_robot.spindexer);

        //PIDFVals pidfSel = PIDFVals.P;

//        ButtonState liftTester =  new ButtonState(gamepad1, ButtonState.Button.a);
//        ButtonState loaderTest = new ButtonState(gamepad1, ButtonState.Button.b);
        ButtonState spinUp = new ButtonState(gamepad2, ButtonState.Button.y);
        ButtonState shootAll = new ButtonState(gamepad2, ButtonState.Button.right_trigger);
        ButtonState shootPattern = new ButtonState(gamepad2, ButtonState.Button.left_trigger);
        ButtonState shootGreen = new ButtonState(gamepad2, ButtonState.Button.left_stick_button);
        ButtonState shootPurple = new ButtonState(gamepad2, ButtonState.Button.right_stick_button);
        ButtonState shootAny = new ButtonState(gamepad2, ButtonState.Button.left_bumper);

//        ButtonState highRollerTest = new ButtonState(gamepad2, ButtonState.Button.y);
//        ButtonState leftMidRollerTest = new ButtonState(gamepad2, ButtonState.Button.x);
//        ButtonState rightMidRollerTest = new ButtonState(gamepad2, ButtonState.Button.b);
//        ButtonState leftLowRollerTest = new ButtonState(gamepad2, ButtonState.Button.dpad_left);
//        ButtonState rightLowRollerTest = new ButtonState(gamepad2, ButtonState.Button.dpad_right);

//        ButtonState intake0 = new ButtonState(gamepad2, ButtonState.Button.dpad_up);
//        ButtonState intake1 = new ButtonState(gamepad2, ButtonState.Button.dpad_right);
//        ButtonState intake2 = new ButtonState(gamepad2, ButtonState.Button.dpad_down);
//
//        ButtonState selectValUp = new ButtonState(gamepad1, ButtonState.Button.dpad_up);
//        ButtonState selectValDown = new ButtonState(gamepad1, ButtonState.Button.dpad_down);
//        ButtonState valUp = new ButtonState(gamepad1, ButtonState.Button.dpad_right);
//        ButtonState valDown = new ButtonState(gamepad1, ButtonState.Button.dpad_left);
        ButtonState spinPwrUp = new ButtonState(gamepad2, ButtonState.Button.dpad_up);
        ButtonState spinPwrDwn = new ButtonState(gamepad2, ButtonState.Button.dpad_down);
//
        ButtonState intakeArtifact = new ButtonState(gamepad2, ButtonState.Button.a);
        ButtonState nextShooterPos = new ButtonState(gamepad2, ButtonState.Button.x);
        ButtonState stopIntake = new ButtonState(gamepad2, ButtonState.Button.b);
        ButtonState reverseIntake = new ButtonState(gamepad2, ButtonState.Button.y);
        ButtonState readColors = new ButtonState(gamepad2, ButtonState.Button.start);

        int shooterPos = 0;


        double powerScale=1;

        PoseVelocity2d driveControl;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        m_robot.intake.stop();
        m_robot.shooter.loadArtifact(0);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Drive Code
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

            if(intakeArtifact.newPress()){
                intakeAction.clearCancel();
                runningActions.add(intakeAction);
            } else if (intakeArtifact.newRelease()){
                intakeAction.cancel();
            }

            if(stopIntake.newPress()){
                m_robot.spindexer.manualSpindexer();
            } else if (stopIntake.newRelease()){
                m_robot.spindexer.stop();
            }

            if(reverseIntake.newPress()){
                m_robot.intake.spitArtifacts();
            } else if (reverseIntake.newRelease()){
                m_robot.intake.stop();
            }

            if(nextShooterPos.newPress()){
                shooterPos ++;
                if (shooterPos > 2) {shooterPos = 0;}
                m_robot.spindexer.moveToShooterPos(shooterPos);
            }

            if(readColors.getCurrentPress()) {
                m_robot.spindexer.getSpindexerPos();

            }

            if(spinUp.newPress()){
                m_robot.shooter.spinUp();
            } else if (spinUp.newRelease()){
                if(!shootAction.isRunning()){
                    m_robot.shooter.idle();
                }
            }

            if(shootAll.newPress()){
                shootAction.clearCancel();
                shootAction.selectShot(ShootAllVariant.ShotType.ShootAll);
                runningActions.add(shootAction);
            } else if (shootAll.newRelease()){
                shootAction.cancel();
            }

            if(shootPattern.newPress()){
                shootAction.clearCancel();
                shootAction.selectShot(ShootAllVariant.ShotType.ShootPattern);
                runningActions.add(shootAction);
            } else if (shootAll.newRelease()){
                shootAction.cancel();
            }

            if(shootGreen.newPress()){
                shootAction.clearCancel();
                shootAction.selectShot(ShootAllVariant.ShotType.ShootGreen);
                runningActions.add(shootAction);
            } else if (shootAll.newRelease()){
                shootAction.cancel();
            }

            if(shootPurple.newPress()){
                shootAction.clearCancel();
                shootAction.selectShot(ShootAllVariant.ShotType.ShootPurple);
                runningActions.add(shootAction);
            } else if (shootAll.newRelease()){
                shootAction.cancel();
            }

            if(shootAny.newPress()){
                shootAction.clearCancel();
                shootAction.selectShot(ShootAllVariant.ShotType.ShootAnySingle);
                runningActions.add(shootAction);
            } else if (shootAll.newRelease()){
                shootAction.cancel();
            }


            /***
             * Test Buttons
             ***/
            //TODO: Remove these when done testing!!

//            if(intake0.newPress()){
//                m_robot.spindexer.moveToIntakePos(0);
//            }
//            if(intake1.newPress()){
//                m_robot.spindexer.moveToIntakePos(1);
//            }
//            if(intake2.newPress()){
//                m_robot.spindexer.moveToIntakePos(2);
//            }




//            if (liftTester.getCurrentPress()) {
//                m_robot.lift.manualClimb(0.2);
//            } else {
//                m_robot.lift.stopLift();
//            }

//            if (loaderTest.newPress()) {
//                m_robot.shooter.loadArtifact(1.0);
//            } else if (loaderTest.newRelease()){
//                m_robot.shooter.loadArtifact(-1.0);
//            }
//            if (liftTester.getCurrentPress()) {
//                m_robot.shooter.loadArtifact(0);
//            }


//            if(shootAll.getCurrentPress()){
//                m_robot.shooter.shoot();
//            } else {
//                m_robot.shooter.stop();
//            }



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



            // Telemetry Updates
            for(int x = 0; x <=2; x++){
                telemetry.addLine()
                        .addData("Slot[", x)
                        .addData("] ->", m_robot.spindexer.getSlotColor(x).name());
            }

            telemetry.addData("Spindexer Pos", m_robot.spindexer.getSpindexerPos());


//            if(selectValUp.newPress()){
//                pidfSel = pidfSel.next();
//            }
//            if(selectValDown.newPress()){
//                pidfSel = pidfSel.prev();
//            }
//            telemetry.addData("Modifying ", pidfSel.name());
//
//            if(valUp.newPress()){
//                switch(pidfSel) {
//                    case P:
//                        pidf.p = pidf.p + 0.1;
//                        break;
//                    case I:
//                        pidf.i = pidf.i + 0.05;
//                        break;
//                    case D:
//                        pidf.d = pidf.d + 0.1;
//                        break;
//                    case F:
//                        pidf.f = pidf.f + 0.1;
//                        break;
//                }
//                m_robot.shooter.setPIDF(pidf);
//            }
//            if(valDown.newPress()){
//                switch(pidfSel) {
//                    case P:
//                        pidf.p = pidf.p - 0.1;
//                        break;
//                    case I:
//                        pidf.i = pidf.i - 0.05;
//                        break;
//                    case D:
//                        pidf.d = pidf.d - 0.1;
//                        break;
//                    case F:
//                        pidf.f = pidf.f - 0.1;
//                        break;
//                }
//                m_robot.shooter.setPIDF(pidf);
//            }
//            if(spinPwrUp.newPress()){
//                m_robot.spindexer.spinPwr = m_robot.spindexer.spinPwr +0.05;
//                if(m_robot.spindexer.spinPwr > 1.0) {m_robot.spindexer.spinPwr = 0.05;}
//            }
//            if (spinPwrDwn.newPress()) {
//                m_robot.spindexer.spinPwr = m_robot.spindexer.spinPwr -0.05;
//                if(m_robot.spindexer.spinPwr < 0.0) {m_robot.spindexer.spinPwr = 1.00;
//            }
            m_robot.shooter.setPIDF(pidf);
            telemetry.addData("Spin Power = ", m_robot.spindexer.spinPwr);
            telemetry.addData("PIDF = ", m_robot.shooter.showPIDFVals());




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
