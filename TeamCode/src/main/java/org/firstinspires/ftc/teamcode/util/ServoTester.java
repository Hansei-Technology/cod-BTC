package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTester extends LinearOpMode {

    Servo servo;
    public static String port = "";
    public static double pos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {


        servo = hardwareMap.get(Servo.class, port);
        waitForStart();
        while (opModeIsActive()){
            servo.setPosition(pos);
        }
    }
}
