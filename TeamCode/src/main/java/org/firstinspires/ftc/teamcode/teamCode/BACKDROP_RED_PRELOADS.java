package org.firstinspires.ftc.teamcode.teamCode;

import androidx.annotation.ColorRes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Config
@Autonomous
public class BACKDROP_RED_PRELOADS extends LinearOpMode {

    enum cases {
        LEFT,
        MID,
        RIGHT
    }
    cases result = cases.RIGHT;

    public static double RIGHT_PURPLE_X = 13, RIGHT_PURPLE_Y = -60, RIGHT_PURPLE_ANGLE = 70;
    Pose2d RIGHT_PURPLE = new Pose2d(RIGHT_PURPLE_X, RIGHT_PURPLE_Y, Math.toRadians(RIGHT_PURPLE_ANGLE));

    public static double LEFT_PURPLE_X = 16.5, LEFT_PURPLE_Y = -51, LEFT_PURPLE_ANGLE = 120;
    Pose2d LEFT_PURPLE = new Pose2d(LEFT_PURPLE_X, LEFT_PURPLE_Y, Math.toRadians(LEFT_PURPLE_ANGLE));

    public static double MID_PURPLE_X = 14, MID_PURPLE_Y = -51, MID_PURPLE_ANGLE = 102;
    Pose2d MID_PURPLE = new Pose2d(MID_PURPLE_X, MID_PURPLE_Y, Math.toRadians(MID_PURPLE_ANGLE));


    public static double LEFT_YELLOW_X = 38, LEFT_YELLOW_Y = -26, LEFT_YELLOW_ANGLE = 0;
    Pose2d LEFT_YELLOW = new Pose2d(LEFT_YELLOW_X, LEFT_YELLOW_Y, Math.toRadians(LEFT_YELLOW_ANGLE));

    public static double RIGHT_YELLOW_X = 50, RIGHT_YELLOW_Y = -28, RIGHT_YELLOW_ANGLE = 0;
    Pose2d RIGHT_YELLOW = new Pose2d(RIGHT_YELLOW_X, RIGHT_YELLOW_Y, Math.toRadians(RIGHT_YELLOW_ANGLE));

    public static double MID_YELLOW_X = 42, MID_YELLOW_Y = -35, MID_YELLOW_ANGLE = 0;
    Pose2d MID_YELLOW = new Pose2d(MID_YELLOW_X, MID_YELLOW_Y, Math.toRadians(MID_YELLOW_ANGLE));

    public static double START_POSE_X = 14, START_POSE_Y = -65, START_POSE_ANGLE = 90;
    Pose2d START_POSE = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));

    public static double PARK_X = 35, PARK_Y = -5, PARK_ANGLE = 70;
    Pose2d PARK = new Pose2d(PARK_X, PARK_Y, Math.toRadians(PARK_ANGLE));



    enum State {
        PRELOAD_LINE,
        YELLOW_PIXEL,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;
    ElapsedTime timerBabi;
    Boolean babi;

    @Override
    public void runOpMode() throws InterruptedException {

        timerBabi = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSE);
        LiftController liftController = new LiftController(hardwareMap);
        ClawController clawController = new ClawController(hardwareMap);
        JointController jointController = new JointController(hardwareMap);
        ArmController armController  =  new ArmController(hardwareMap);
        armController.goMid();
        jointController.goToUp();
        clawController.toggleLeft();
        clawController.toggleRight();
        liftController.update();
        armController.update();

        ElapsedTime timeLeft = new ElapsedTime();


        Trajectory preloadLineLeftTraj = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(LEFT_PURPLE) //left
                .build();
        Trajectory preloadLineMidTraj = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(MID_PURPLE) //mid
                .build();
        Trajectory preloadLineRightTraj = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(RIGHT_PURPLE) //right
                .build();
        Trajectory yellowPixelLeftTraj = drive.trajectoryBuilder(preloadLineLeftTraj.end()) // left
                .lineToLinearHeading(LEFT_YELLOW)
                .build();
        Trajectory yellowPixelMidTraj = drive.trajectoryBuilder(preloadLineMidTraj.end()) //mid
                .lineToLinearHeading(MID_YELLOW)
                .build();
        Trajectory yellowPixelRightTraj = drive.trajectoryBuilder(preloadLineRightTraj.end()) //Right
                .lineToLinearHeading(RIGHT_YELLOW)
                .build();


        waitForStart();

        timeLeft.reset();
        if (isStopRequested()) return;
        currentState = State.PRELOAD_LINE;
        switch (result) {
            case LEFT:
                drive.followTrajectoryAsync(preloadLineLeftTraj);
                break;
            case MID:
                drive.followTrajectoryAsync(preloadLineMidTraj);
                break;
            case RIGHT:
                drive.followTrajectoryAsync(preloadLineRightTraj);
                break;
        }
        babi = false;

        while (opModeIsActive() && !isStopRequested())
        {
            switch (currentState)
            {
                case PRELOAD_LINE:
                {
                    armController.goDown();
                    liftController.goTOPos(liftController.MaxPoz);
                    jointController.goToDown();
                    if (!drive.isBusy())
                    {
                        if(!babi) timerBabi.reset();
                        clawController.openRight();
                        babi = true;
                        if(timerBabi.milliseconds() > 200) {
                            currentState = State.YELLOW_PIXEL;
                            switch (result) {
                                case RIGHT:
                                    drive.followTrajectoryAsync(yellowPixelRightTraj);
                                    break;
                                case MID:
                                    drive.followTrajectoryAsync(yellowPixelMidTraj);
                                    break;
                                case LEFT:
                                    drive.followTrajectoryAsync(yellowPixelLeftTraj);
                                    break;
                            }
                            armController.goMid();
                            liftController.goMid();
                            jointController.goToMid();
                            babi = false;
                        }
                    }
                    break;
                }
                case YELLOW_PIXEL:
                {
                    if(!drive.isBusy())
                    {
                        if(!babi) timerBabi.reset();
                        babi = true;
                        clawController.openLeft();
                        if(timerBabi.milliseconds() > 200) {
                            liftController.goDown();
                            currentState = State.PARK;
                            Trajectory parkTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToConstantHeading(PARK.vec())
                                    .build();
                            drive.followTrajectoryAsync(parkTraj);

                            liftController.goTOPos(0);
                        }
                    }
                    break;
                }
                case PARK:
                {
                    if(!drive.isBusy())
                    {
                        armController.goMid();
                        jointController.goToDown();
                    }
                    break;
                }
                case IDLE:
                    break;
            }


            liftController.update();
            armController.update();
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("Time Left", timeLeft.seconds());
            telemetry.update();
        }
    }
}
