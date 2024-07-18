package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ArmController{
    double power;
    public int target = 0;
    public int currentPos;
    public static int MaxPoz = 900;
    public static int MidPoz = 550;
    public static int MinPoz = 150;
    public static int topStackLevelPos = 350;
    DcMotorEx armMotor;
    public ArmController(HardwareMap map)
    {
        armMotor = map.get(DcMotorEx.class, "m0e");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void goUp()
    {
        target = MaxPoz;
        armMotor.setTargetPosition(target);
    }
    public void goDown()
    {
        target = MinPoz;
        armMotor.setTargetPosition(target);
    }
    public void goMid()
    {
        target = MidPoz;
        armMotor.setTargetPosition(target);
    }
    public void goToCollect(int stackLevel){
        if(stackLevel==5)
        {
            target = topStackLevelPos;
        } else
        {
            int level = 5 - topStackLevelPos;
            target = 10*level;
        }
        armMotor.setTargetPosition(target);

    }

    public void update()
    {
        currentPos = armMotor.getCurrentPosition();
        if(currentPos - target > 50) power = 0.4;
        else power = 1;
        armMotor.setPower(power);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
