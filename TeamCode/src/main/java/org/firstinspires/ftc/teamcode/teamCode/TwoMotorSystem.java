package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TwoMotorSystem extends Thread{
    public double power = 0;
    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public TwoMotorSystem(DcMotorEx motor1, DcMotorEx motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    public void run() {
        while(true) {
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }
}
