package org.firstinspires.ftc.teamcode.TeamCode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Config
public class Auto2Specimene extends LinearOpMode {

    public static double PRELOAD_x = 13, PRELOAD_y = 0, PRELOAD_heading = 0;
    public static double SAFE_x = 13, SAFE_y = -16, SAFE_heading = 270;
    public static double SAFE2_x = 36, SAFE2_y = -23, SAFE2_heading = 180;
    public static double ELEM1_x = 28, ELEM1_y = -40, ELEM1_heading = 180;
    public static double HUMAN_x = 7, HUMAN_y = -40, HUMAN_heading = 180;
    public static double SAFE3_x = 13, SAFE3_y = -40, SAFE3_heading = 180;
    public static double SPECIMEN_x = 7, SPECIMIEN_y = -40  , SPECIMEN_heading = 180;
    public static double PRELOAD2_x = 13, PRELOAD2_y = 0, PRELOAD2_heading = 0;

    public void runOpMode() throws InterruptedException{



        Oriz oriz = new Oriz(hardwareMap);
        Vert vert = new Vert(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        Joint joint = new Joint(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d PRELOAD = new Pose2d(PRELOAD_x, PRELOAD_y, Math.toRadians(PRELOAD_heading));
        Pose2d SAFE = new Pose2d(SAFE_x, SAFE_y, Math.toRadians(SAFE_heading));
        Pose2d SAFE2 = new Pose2d(SAFE2_x, SAFE2_y, Math.toRadians(SAFE2_heading));
        Pose2d ELEM1 = new Pose2d(ELEM1_x, ELEM1_y, Math.toRadians(ELEM1_heading));
        Pose2d HUMAN = new Pose2d(HUMAN_x, HUMAN_y, Math.toRadians(HUMAN_heading));
        Pose2d SAFE3 = new Pose2d(SAFE3_x, SAFE3_y, Math.toRadians(SAFE3_heading));
        Pose2d SPECIMEN = new Pose2d(SPECIMEN_x, SPECIMIEN_y, Math.toRadians(SPECIMEN_heading));
        Pose2d PRELOAD2 = new Pose2d(PRELOAD2_x, PRELOAD2_y, Math.toRadians(PRELOAD2_heading));


        oriz.goToPoz(0.4);
        joint.goToLevel();
        cleste.close();

        waitForStart();


        vert.goToHigh();

        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .waitSeconds(0.5)
                .lineToLinearHeading(PRELOAD)
                .addTemporalMarker( () -> {
                    vert.goToMid();
                })
                .waitSeconds(0)
                .waitSeconds(0.4)
                        .waitSeconds(0)
                        .addTemporalMarker( () -> {
                            cleste.open();
                        })
                        .waitSeconds(0.5)
                        .waitSeconds(0)
                        .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    vert.goDown();
                })
                .waitSeconds(0)
                .lineToLinearHeading(SAFE)
                .lineToLinearHeading(SAFE2)
                .lineToLinearHeading(ELEM1)
                .lineToLinearHeading(HUMAN)
                        .lineToLinearHeading(SAFE3)
                        .waitSeconds(1.5)
                        .lineToLinearHeading(SPECIMEN)
                        .addTemporalMarker( () -> {
                            vert.goToCapuUrsului();
                        })
                        .addTemporalMarker( () -> {
                            vert.goToHigh();
                        })
                        .waitSeconds(0.1)
                        .waitSeconds(0)
                        .addTemporalMarker( () -> {
                            cleste.close();
                        })
                        .lineToLinearHeading(PRELOAD2)
                .addTemporalMarker( () -> {
                    vert.goToMid();
                })
                .waitSeconds(0)
                .waitSeconds(0.5)
                        .waitSeconds(0)
                        .addTemporalMarker( () -> {
                            cleste.open();
                        })
                .waitSeconds(0.5)
                .waitSeconds(0)
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    vert.goDown();
                })
                .build());

        while(opModeIsActive()) {
            vert.update();
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
}
