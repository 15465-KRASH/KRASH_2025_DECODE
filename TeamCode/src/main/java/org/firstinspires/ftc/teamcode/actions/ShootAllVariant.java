package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.Shooter;
import org.firstinspires.ftc.teamcode.classes.Spindexer;

public class ShootAllVariant implements Action {
    private boolean initialized = false;
    private boolean ammoRdy = false;
    private boolean running = true;
    private boolean atSpeed = false;
    private boolean safeMove = false;
    private boolean canceled = false;
    private boolean lastshot = false;
    private int targetSlot = -1;
    private double shotTime = 0.75;
    private boolean waiting = false;
    
    private int totalShots = 3;
    private int currentShot = 0;

    private Spindexer.DetectedColor colorTarget = Spindexer.DetectedColor.ANY;

    //Timers
    private double firstShotHoldMin = 0.5;
    private double firstShotWait = 0.5;
    private double secondShotWait = 0.5;
    private double finalShotWait = 0.5;

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private Shooter shooter;
    private Spindexer spindexer;

    public enum ShotType{
        ShootGreen,
        ShootPurple,
        ShootAnySingle,
        ShootPattern,
        ShootAll
    }

    private ShotType shotType = ShotType.ShootAll;

    public ShootAllVariant(Shooter shooter, Spindexer spindexer){
        this.shooter = shooter;
        this.spindexer = spindexer;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!canceled) {
            if (!initialized) {
                getNextColor(1);
                shooter.spinUp();
                timer.reset();
                lastshot = false;
                waiting = true;
                safeMove = false;
                initialized = true;
                running = true;
                currentShot = 1;
                targetSlot = spindexer.gotoClosestFullShooter(colorTarget);
            }

            if(targetSlot == -1 && !lastshot){
                cleanup();
//                packet.put("No full slot", 0);
                return false;
            }

            atSpeed = shooter.atSpeed();

            if(lastshot){
                waiting = timer.seconds() <= finalShotWait;
                if(!waiting){
                    cleanup();
                    return false;
                }
            } else {
                if (currentShot == 1 && waiting) {
                    waiting = timer.seconds() <= firstShotHoldMin;
                } else if (currentShot == 2) {
                    waiting = timer.seconds() <= firstShotWait;
                    if (!waiting) {
                        spindexer.moveToShooterPos(targetSlot);
                    }
                } else if (currentShot == 3) {
                    waiting = timer.seconds() <= secondShotWait;
                    if (!waiting) {
                        spindexer.moveToShooterPos(targetSlot);
                    }
                }
            }
            
            if (atSpeed && !waiting) {
                if (spindexer.spindexerAtTarget()) {
                    packet.put("Executing launch", 0);
                    packet.put("Target Slot: ", targetSlot);

                    shooter.loadArtifact(1.0);
                    spindexer.setSlot(targetSlot, Spindexer.DetectedColor.NONE);

                    packet.put("Color 0: ", spindexer.spindexerSlots[0].name());
                    packet.put("Color 1: ", spindexer.spindexerSlots[1].name());
                    packet.put("Color 2: ", spindexer.spindexerSlots[2].name());

                    if(currentShot < totalShots){
                        currentShot ++;
                        getNextColor(currentShot);
                        targetSlot = spindexer.findFullShooterSlot(colorTarget);
                    } else {
                        targetSlot = -1;
                    }

                    if(targetSlot == -1){
                        lastshot = true;
                    }

                    timer.reset();

                    packet.put("Next target slot", targetSlot);
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

    public void getNextColor(int shotNumber) {
        switch (shotType) {
            case ShootGreen:
                colorTarget = Spindexer.DetectedColor.GREEN;
                totalShots = 1;
                break;
            case ShootPurple:
                colorTarget = Spindexer.DetectedColor.PURPLE;
                totalShots = 1;
                break;
            case ShootAnySingle:
                colorTarget = Spindexer.DetectedColor.ANY;
                totalShots = 1;
                break;
            case ShootPattern:
                // TODO: Replace with method supplying pattern color order
                colorTarget = Spindexer.DetectedColor.ANY;
                totalShots = 3;
                break;
            case ShootAll:
                colorTarget = Spindexer.DetectedColor.ANY;
                totalShots = 3;
                break;
        }
    }

    public void cancel(){
        canceled = true;
    }

    public void cleanup(){
        shooter.idle();
        shooter.loadArtifact(0);
        initialized = false;
        running = false;
        safeMove  = false;
        lastshot = false;
    }

    public void clearCancel(){
        canceled = false;
    }

    public void selectShot(ShotType type){
        shotType = type;
    }

    public boolean isRunning(){
        return running;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}