package org.firstinspires.ftc.teamcode.teamCode;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OrizController{
    Servo servo;
    public static double Closed = 0.47; //0.4
    public static double Open = 0.58;
    public static double speed = 0.001;

    double currentPos = 0;

    public OrizController(HardwareMap map) {
        servo = map.get(Servo.class, "s0");
    }

    public void close() {
        servo.setPosition(Closed);
        currentPos = Closed;
    }

    public void open() {
        servo.setPosition(Open);
        currentPos = Open;
    }

    public void goToPos(double pos) {
        if(pos >= 0.4 && pos <= Open) {
            servo.setPosition(pos);
            currentPos = pos;
        }
    }

    public void extend() {
        goToPos(currentPos + speed);
    }

    public void retract() {
        goToPos(currentPos - speed);
    }
}
