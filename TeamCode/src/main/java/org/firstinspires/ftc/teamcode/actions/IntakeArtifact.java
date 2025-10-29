package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Spindexer;

public class IntakeArtifact implements Action {
    private boolean initialized = false;
    private boolean running = true;
    private int targetSlot = -1;

    private Intake intake;
    private Spindexer spindexer;

    public IntakeArtifact(Intake intake, Spindexer spindexer){
        this.intake = intake;
        this.spindexer = spindexer;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        targetSlot = spindexer.gotoClosestEmptyIntake();
        if (!initialized && targetSlot != -1) {
            intake.intakeArtifact();
            initialized = true;
            running = true;
        } else if (!initialized){
            intake.stop();
            running = false;
        }

        if(running) {
            if (spindexer.spindexerAtTarget() && spindexer.isIntakeSlotFull()) {
                spindexer.setSlot(targetSlot, spindexer.getIntakeColor());
                targetSlot = spindexer.gotoClosestEmptyIntake();
                if (targetSlot != -1) {
                    intake.intakeArtifact();
                } else {
                    intake.stop();
                    running = false;
                }
            }
        }


//        packet.put("shooterVelocity", vel);
        return running;

    }

    public Action intakeArtifact(Intake intake, Spindexer spindexer){
        return new IntakeArtifact(intake, spindexer);
    }
}