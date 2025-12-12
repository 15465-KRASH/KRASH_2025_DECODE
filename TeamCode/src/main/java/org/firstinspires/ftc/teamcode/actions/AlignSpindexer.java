package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.Spindexer;

public class AlignSpindexer implements Action {
    private Telemetry telemetry;

    private boolean initialized = false;
    private boolean running = true;
    private boolean waiting = false;
    private boolean canceled = false;
    private int targetSlot = 0;

    private double holdTime = 0.1;

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private Spindexer spindexer;

    public AlignSpindexer(Spindexer spindexer, Telemetry telemetry){
        this.telemetry = telemetry;
        this.spindexer = spindexer;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!canceled) {
            if (!initialized) {
                spindexer.gotoAlignPos(targetSlot);
                initialized = true;
                running = true;
                waiting = true;
                timer.reset();
            }

            waiting = timer.seconds() < holdTime;

            if (running && spindexer.atTarget() && !waiting) {
                telemetry.addData("Manual Move:", 0);
                telemetry.addData("MagSensor", spindexer.readMagSensor());
                spindexer.manualSpindexer();
                if(spindexer.readMagSensor()){
                    spindexer.stop();
                    spindexer.resetPos();
                }
            }
            return running;
        } else {
            canceled = false;
            cleanup();
            return false;
        }
//        packet.put("shooterVelocity", vel);

    }

    public void cancel(){
        canceled = true;
    }

    public void cleanup(){
        initialized = false;
        running = false;
        spindexer.stop();
    }

    public boolean isRunning(){
        return running;
    }


    public void clearCancel(){
        canceled = false;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }



}