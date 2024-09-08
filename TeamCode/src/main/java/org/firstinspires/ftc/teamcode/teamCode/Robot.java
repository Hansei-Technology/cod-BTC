package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot extends Thread {

        public LinearOpMode opMode;
        public ArmController arm;
        public LiftController lift;
        public JointController2Servo joint;
        public ClawController2Servo claw;

        public void run() {
            while (!isInterrupted()) {
                arm.update();
                lift.update();
            }
        }

        public Robot(LinearOpMode opMode) {
            HardwareMap map = opMode.hardwareMap;
            this.opMode = opMode;
            arm = new ArmController (map);
            lift = new LiftController (map);
            joint = new JointController2Servo(map);
            claw = new ClawController2Servo(map);

        }
    }
