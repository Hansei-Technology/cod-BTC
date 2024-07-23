package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamCode.ArmController;
import org.firstinspires.ftc.teamcode.teamCode.JointController;
import org.firstinspires.ftc.teamcode.teamCode.LiftController;

@TeleOp
@Config
public class InitPoseFinder extends LinearOpMode {
    LiftController lift;
    ArmController arm;
    JointController joint;

    public static int liftPoz = 0, armPoz = 0;
    public static double jointPoz = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = new LiftController(hardwareMap);
        arm = new ArmController(hardwareMap);
        joint = new JointController(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            lift.goTOPos(liftPoz);
            arm.goToPoz(armPoz);
            joint.goToPoz(jointPoz);

            arm.update();
            lift.update();
        }
    }
}
