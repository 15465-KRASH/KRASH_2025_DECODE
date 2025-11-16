package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lights {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private Servo leftLight, rightLight;

    private boolean onCycle = true;
    private double sparkleTimer = 0.05;
    private double rainbowSetting = 0.277;
    private boolean rainbowDirection = true;

    public Lights (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        leftLight = hardwareMap.get(Servo.class, "leftLight");
        rightLight = hardwareMap.get(Servo.class, "leftLight");

        timer.reset();
    }

    public void setYellow(){
        leftLight.setPosition(0.388);
        rightLight.setPosition(0.388);
    }

    public void setOff(){
        leftLight.setPosition(0);
        rightLight.setPosition(0);
    }

    public void set(double setPoint){
        leftLight.setPosition(setPoint);
        rightLight.setPosition(setPoint);
    }

    public void sparkle(){
        if(timer.seconds() >= sparkleTimer){
            onCycle = !onCycle;
            if(onCycle){
                setYellow();
            } else {
                setOff();
            }
        }
    }

    public void rainbow(){
        double rainbowStep = 0.001;
        set(rainbowSetting);
        if(rainbowDirection && rainbowSetting < 0.722){
            rainbowSetting = rainbowSetting + rainbowStep;
        } else if (!rainbowDirection && rainbowSetting > 0.277){
            rainbowSetting = rainbowSetting - rainbowStep;
        }
        if (rainbowSetting <= 0.277){
            rainbowDirection = true;
            rainbowSetting = 0.277;
        } else if (rainbowSetting >= 0.722) {
            rainbowDirection = false;
            rainbowSetting = 0.722;
        }
//        setYellow();
    }


}
