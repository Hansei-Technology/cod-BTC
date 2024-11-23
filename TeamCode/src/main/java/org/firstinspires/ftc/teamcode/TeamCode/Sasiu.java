package org.firstinspires.ftc.teamcode.TeamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Sasiu {
    DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public static double speed = 0.6;
    public Sasiu(HardwareMap map)
    {
        leftFront = map.get(DcMotorEx.class, "m1");
        rightFront = map.get(DcMotorEx.class, "m3");
        leftBack = map.get(DcMotorEx.class, "m0");
        rightBack = map.get(DcMotorEx.class, "m2");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(Gamepad g)
    {
        leftFront.setPower((-g.left_stick_y + g.left_stick_x + g.right_stick_x) * speed);
        rightFront.setPower((-g.left_stick_y - g.left_stick_x - g.right_stick_x) * speed);
        leftBack.setPower((-g.left_stick_y - g.left_stick_x + g.right_stick_x) * speed);
        rightBack.setPower((-g.left_stick_y + g.left_stick_x - g.right_stick_x) * speed);

    }

}