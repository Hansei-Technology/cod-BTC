package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class JointController2Servo {
    Servo left, right;
    public static double downPoz = 0.6;
    public static double midPoz = 0.5;
    public static double upPoz = 0.32;

    public JointController2Servo(HardwareMap map) {
        left = map.get(Servo.class, "s0");
        right = map.get(Servo.class, "s4");
    }

    public void goToMid()
    {
        left.setPosition(midPoz);
        right.setPosition(midPoz);
    }
    public void goToDown()
    {
        left.setPosition(downPoz);
        right.setPosition(downPoz);
    }
    public void goToLevel()
    {
        left.setPosition(upPoz);
        right.setPosition(upPoz);
    }

    public void goToPoz(double poz) {
        left.setPosition(poz);
        right.setPosition(poz);
    }
}
