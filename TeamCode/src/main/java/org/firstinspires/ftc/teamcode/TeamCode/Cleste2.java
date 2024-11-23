package org.firstinspires.ftc.teamcode.TeamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Cleste2 {
    Servo servo;
    public static double downPoz = 0.3;
    public static double midPoz = 0.42;
    public static double upPoz = 0.6;

    public Cleste2(HardwareMap map) {
        servo = map.get(Servo.class, "s2");
    }

    public enum Status{
        OPEN, CLOSED;
    }

    Status status = Status.OPEN;

    public void Toggle(){
        if(status == Status.OPEN){
            status = Status.CLOSED;
            servo.setPosition(downPoz);
        }
        else{
            status = Status.OPEN;
            servo.setPosition(upPoz);
        }
    }

    public void goToMid()
    {
        servo.setPosition(midPoz);
    }
    public void goToDown()
    {
        servo.setPosition(downPoz);
    }
    public void goToLevel()
    {
        servo.setPosition(upPoz);
    }

    public void goToPoz(double poz) {
        servo.setPosition(poz);
    }
}