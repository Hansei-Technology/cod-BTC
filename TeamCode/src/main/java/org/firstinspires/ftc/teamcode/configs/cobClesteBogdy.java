package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp
@Config
public class cobClesteBogdy extends LinearOpMode {
    Servo clesteL, clesteR, rot;

    public static double LOpen = 0.5;
    public static double LClosed = 0.5;
    public static double ROpen = 0.5;
    public static double RClosed = 0.5;

    public static double level = 0.5, left = 0.5, right = 0.5;
    public static String cLName = "", cRName = "", rName = "";
    @Override
    public void runOpMode() throws InterruptedException {
        clesteL = hardwareMap.get(Servo.class, cLName);
        clesteR = hardwareMap.get(Servo.class, cRName);
        rot = hardwareMap.get(Servo.class, rName);
        StickyGamepad g = new StickyGamepad(gamepad1, this);
        waitForStart();
        while(opModeIsActive())
        {
            if(g.right_bumper) {
                if(clesteL.getPosition() == LOpen) clesteL.setPosition(LClosed);
                else clesteL.setPosition(LOpen);
            }
            if(g.left_bumper) {
                if(clesteR.getPosition() == ROpen) clesteR.setPosition(RClosed);
                else clesteR.setPosition(ROpen);
            }

            if(g.a) rot.setPosition(level);
            if(g.b) rot.setPosition(left);
            if(g.y) rot.setPosition(right);
            g.update();
        }

    }
}
