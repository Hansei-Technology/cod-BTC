package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Cleste1 {

    Servo motor;
    public double deschis = 0.3;
    public double inchis = 0.7;

    public Cleste1(HardwareMap map){
        motor = map.get(Servo.class, "s3");
    }

    public enum Status{
        DESCHIS, INCHIS
    }

    Status status = Status.DESCHIS;

    public void Toggle(){
        if(status == Status.DESCHIS){
            status = Status.INCHIS;
            motor.setPosition(inchis);
        }
        else {
            status = Status.DESCHIS;
            motor.setPosition(deschis);
        }
    }

    public Status getStatus() {
        return status;
    }

    public void setStatus(Status status) {
        this.status = status;
    }

    public void open(){
        status = Status.DESCHIS;
        motor.setPosition(deschis);
    }

    public void close(){
        status = Status.INCHIS;
        motor.setPosition(inchis);
    }

    public void setPos(double pos){
        motor.setPosition(pos);
    }
}
