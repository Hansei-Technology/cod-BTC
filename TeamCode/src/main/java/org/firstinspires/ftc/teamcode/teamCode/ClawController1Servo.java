package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawController1Servo {
    Servo left;
    double Closed = 0.7;
    double Open = 0.3;

    public enum Status
    {
        OPEN,
        CLOSED
    }

    Status leftStatus = Status.OPEN;

    public ClawController1Servo(HardwareMap map) {
        left = map.get(Servo.class, "s3");
    }

    public void toggle() {
        if(leftStatus == Status.OPEN) {
            leftStatus = Status.CLOSED;
            left.setPosition(Closed);
        } else {
            leftStatus = Status.OPEN;
            left.setPosition(Open);
        }
    }

    public void open() {
        leftStatus = Status.OPEN;
        left.setPosition(Open);
    }

    public void close() {
        leftStatus = Status.CLOSED;
        left.setPosition(Closed);
    }

}
