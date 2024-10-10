package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    boolean bratEJos = false;
    StickyGamepad sticky1;
    StickyGamepad sticky2;
    ChassisController sasiu;
    JointController1Servo joint;
    LiftController lift;
    OrizController oriz;

    ClawController1Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        oriz = new OrizController(hardwareMap);
        sasiu = new ChassisController(hardwareMap);
        lift = new LiftController(hardwareMap);
        claw = new ClawController1Servo(hardwareMap);
        joint = new JointController1Servo(hardwareMap);
        joint.goToMid();
        sticky1 = new StickyGamepad(gamepad1, this);
        sticky2 = new StickyGamepad(gamepad2, this);

        waitForStart();
        while(opModeIsActive())
        {

            sasiu.move(gamepad1);
            lift.update();
            sticky1.update();
            sticky2.update();

//            if(lift.currentPos > 400 && bratEJos)
//            {
//                arm.target = 500;
//            }


            if(gamepad1.a)
            {
                bratEJos = true;
                joint.goToLevel();
                oriz.close();
                lift.goDown();
            }
            if(gamepad1.b)
            {
                bratEJos = false;
                joint.goToLevel();
                oriz.close();
                lift.goToMid();
            }
            if(gamepad1.y)
            {
                bratEJos = false;
                lift.goToHigh();
                oriz.close();
                joint.goToLevel();
            }

            if(gamepad1.x) {
                joint.goToMid();
            }

            if (gamepad1.left_stick_y != 0) {
                lift.setPower(gamepad1.left_stick_y);
            }

            if (gamepad1.dpad_down) {
                oriz.close(); }
            if (gamepad1.dpad_up) {
                joint.goToDown();
                oriz.open(); }

            if(gamepad1.dpad_right) {
                lift.goToPoz(lift.GRAB_POS);
            }

            if (sticky1.left_bumper) {
                claw.toggle();
            }

            if(gamepad1.right_bumper) {
                joint.goToDown();
                oriz.open();
            }

            lift.update();
            telemetry.addData("glis: ", lift.position);
            telemetry.update();
        }
    }
}
