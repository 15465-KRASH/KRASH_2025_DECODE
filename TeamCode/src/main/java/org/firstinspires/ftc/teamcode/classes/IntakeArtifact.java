package org.firstinspires.ftc.teamcode.classes;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArtifact {

    private Telemetry telemetry;

    private Spindexer spindexer;
    private Intake intake;

    public IntakeArtifact(Intake intake, Spindexer spindexer, Telemetry telemetry){
        this.intake = intake;
        this.spindexer = spindexer;
        this.telemetry = telemetry;
    }

}
