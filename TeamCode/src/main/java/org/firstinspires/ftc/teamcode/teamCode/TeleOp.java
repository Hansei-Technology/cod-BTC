package org.firstinspires.ftc.teamcode.TeamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.TeamCode.Vert;

import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Config
public class TeleOp extends LinearOpMode {

    Cleste1 cleste1;
    Sasiu sasiu;
    Oriz oriz;
    Vert vert;
    StickyGamepad sticky1;
    StickyGamepad sticky2;
    Cleste2 cleste2;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sasiu = new Sasiu(hardwareMap);
        cleste1 = new Cleste1(hardwareMap);
        oriz = new Oriz(hardwareMap);
        sticky1 = new StickyGamepad(gamepad1, this);
        sticky2 = new StickyGamepad(gamepad2, this);
        cleste2 = new Cleste2(hardwareMap);
        vert = new Vert(hardwareMap);


        waitForStart();

        cleste2.goToLevel();
        cleste1.open();
        
        while(opModeIsActive()){

            sasiu.move(gamepad1);
            vert.update();
            sticky1.update();
            sticky2.update();

            if(gamepad1.right_trigger > 0.1){
                oriz.Open();
            }

            if(gamepad1.left_trigger > 0.1){
                oriz.Close();
            }

            if(sticky1.left_bumper){
                cleste1.Toggle();
            }

            if(gamepad1.a){
                oriz.Close();
                cleste2.goToLevel();
                vert.goDown();
            }

            if(gamepad1.b){
                oriz.Close();
                cleste2.goToLevel();
                vert.goToMid();
            }

            if(gamepad1.y){
                oriz.Close();
                cleste2.goToLevel();
                vert.goToHigh();
            }

            if(sticky1.right_bumper){
                cleste2.Toggle();
            }

            if(gamepad1.x){
                vert.goToPoz(vert.GRAB_POS);
            }

            if(gamepad1.dpad_right){
                vert.goToCapuUrsului();
            }

            if(gamepad1.dpad_left){
                cleste2.goToMid();
            }


            vert.update();
            telemetry.addData("glis: ", vert.position);
            telemetry.update();
        }

    }
}
