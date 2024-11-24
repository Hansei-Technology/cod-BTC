package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Joint {

    public static double downPos = 0.3;
    public static double midPos = 0.42;
    public static double levelPos = 0.6;
    Servo servo;

    public Joint(HardwareMap map){
        servo = map.get(Servo.class, "s1");
    }

    public enum Status{
        LEVEL, DOWN, MID
    }

    Status status = Status.LEVEL;

    public void toggle(){
        if(status == Status.LEVEL){
            status = Status.DOWN;
            servo.setPosition(downPos);
        }
        else if(status == Status.DOWN || status == Status.MID){
            status = Status.LEVEL;
            servo.setPosition(levelPos);
        }
    }

    public void goDown(){
        servo.setPosition(downPos);
    }

    public void goToLevel(){
        servo.setPosition(levelPos);
    }

    public void goToMid(){
        servo.setPosition(midPos);
    }
}
