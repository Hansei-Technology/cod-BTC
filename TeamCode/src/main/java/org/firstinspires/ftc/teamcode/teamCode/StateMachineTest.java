package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.StateMachine;


public class StateMachineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        StateMachine machine = new StateMachineBuilder();
        
        waitForStart();
        while(opModeIsActive())
        {

        }
    }
}
