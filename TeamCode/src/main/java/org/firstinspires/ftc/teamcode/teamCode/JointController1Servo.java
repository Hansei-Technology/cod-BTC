package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class JointController1Servo {
    Servo left;
    public static double downPoz = 0.3;
    public static double midPoz = 0.42;
    public static double upPoz = 0.6;

    public JointController1Servo(HardwareMap map) {
        left = map.get(Servo.class, "s2");
    }

    public void goToMid()
    {
        left.setPosition(midPoz);
    }
    public void goToDown()
    {
        left.setPosition(downPoz);
    }
    public void goToLevel()
    {
        left.setPosition(upPoz);
    }

    public void goToPoz(double poz) {
        left.setPosition(poz);
    }
}
