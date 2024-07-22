package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class STACK_RED_PRELOADS extends LinearOpMode {

    enum State {
        PRELOAD_LINE,
        YELLOW_PIXEL,
        PARK,
        IDLE,
        SAFE1,
        SAFE2
    }

    State currentState = State.IDLE;

    public static double RIGHT_PURPLE_X = -38, RIGHT_PURPLE_Y = -54, RIGHT_PURPLE_ANGLE = 124;
    Pose2d RIGHT_PURPLE = new Pose2d(RIGHT_PURPLE_X, RIGHT_PURPLE_Y, Math.toRadians(RIGHT_PURPLE_ANGLE));

    public static double LEFT_PURPLE_X = -36, LEFT_PURPLE_Y = -60, LEFT_PURPLE_ANGLE = 70;
    Pose2d LEFT_PURPLE = new Pose2d(LEFT_PURPLE_X, LEFT_PURPLE_Y, Math.toRadians(LEFT_PURPLE_ANGLE));

    public static double MID_PURPLE_X = -45, MID_PURPLE_Y = -0.5, MID_PURPLE_ANGLE = -135;
    Pose2d MID_PURPLE = new Pose2d(MID_PURPLE_X, MID_PURPLE_Y, Math.toRadians(MID_PURPLE_ANGLE));


    public static double LEFT_YELLOW_X = 46.5, LEFT_YELLOW_Y = -20, LEFT_YELLOW_ANGLE = -8;
    Pose2d LEFT_YELLOW = new Pose2d(LEFT_YELLOW_X, LEFT_YELLOW_Y, Math.toRadians(LEFT_YELLOW_ANGLE));

    public static double RIGHT_YELLOW_X = 45.5, RIGHT_YELLOW_Y = -33, RIGHT_YELLOW_ANGLE = 0;
    Pose2d RIGHT_YELLOW = new Pose2d(RIGHT_YELLOW_X, RIGHT_YELLOW_Y, Math.toRadians(RIGHT_YELLOW_ANGLE));

    public static double MID_YELLOW_X = 45, MID_YELLOW_Y = -25.5, MID_YELLOW_ANGLE = 0;
    Pose2d MID_YELLOW = new Pose2d(MID_YELLOW_X, MID_YELLOW_Y, Math.toRadians(MID_YELLOW_ANGLE));

    public static double START_POSE_X = -35.5, START_POSE_Y = -62.5, START_POSE_ANGLE = 90;
    Pose2d START_POSE = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_ANGLE));

    public static double SAFE1_POSE_X = -45, SAFE1_POSE_Y = 3, SAFE1_POSE_ANGLE = 0;
    Pose2d SAFE1_POSE = new Pose2d(SAFE1_POSE_X, SAFE1_POSE_Y, Math.toRadians(SAFE1_POSE_ANGLE));

    public static double SAFE2_POSE_X = 15, SAFE2_POSE_Y = 3, SAFE2_POSE_ANGLE = 0;
    Pose2d SAFE2_POSE = new Pose2d(SAFE2_POSE_X, SAFE2_POSE_Y, Math.toRadians(SAFE2_POSE_ANGLE));
    public static double PARK_X = 42, PARK_Y = -5, PARK_ANGLE = 0;
    Pose2d PARK = new Pose2d(PARK_X, PARK_Y, Math.toRadians(PARK_ANGLE));

    ElapsedTime timerBabi;
    Boolean babi;

    enum random {
        LEFT,
        RIGHT,
        MID
    }
    random result = random.MID;

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

        Trajectory preloadLineLeftTraj = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(LEFT_PURPLE) //left
                .build();
        Trajectory preloadLineMidTraj = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(MID_PURPLE) //mid
                .build();
        Trajectory preloadLineRightTraj = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(RIGHT_PURPLE) //right
                .build();

        Trajectory yellowPixelLeftTraj = drive.trajectoryBuilder(SAFE2_POSE) // left
                .lineToLinearHeading(LEFT_YELLOW)
                .build();
        Trajectory yellowPixelMidTraj = drive.trajectoryBuilder(SAFE2_POSE) //mid
                .lineToLinearHeading(MID_YELLOW)
                .build();
        Trajectory yellowPixelRightTraj = drive.trajectoryBuilder(SAFE2_POSE) //right
                .lineToLinearHeading(RIGHT_YELLOW)
                .build();

        Trajectory safe2Traj = drive.trajectoryBuilder(SAFE1_POSE)
                .lineToLinearHeading(SAFE2_POSE)
                .build();


        ElapsedTime timeLeft = new ElapsedTime();

        waitForStart();

        timeLeft.reset();
        if (isStopRequested()) return;
        currentState = State.PRELOAD_LINE;
        drive.followTrajectoryAsync(preloadLineMidTraj);
        babi = false;

        while (opModeIsActive() && !isStopRequested())
        {
            switch (currentState)
            {
                case PRELOAD_LINE:
                {
                    armController.goDown();
                    if(result == random.MID) liftController.goTOPos(250);
                    else liftController.goTOPos(liftController.MaxPoz);
                    jointController.goToDown();
                    if (!drive.isBusy())
                    {
                        if(!babi) timerBabi.reset();
                        clawController.openRight();
                        babi = true;
                        if(timerBabi.milliseconds() > 200) {
                            currentState = State.SAFE1;
                            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(SAFE1_POSE)
                                    .build();
                            drive.followTrajectoryAsync(traj);
                            liftController.goTOPos(0);
                            armController.goMid();
                        }
                    }
                    break;
                }
                case SAFE1:
                {
                    if(!drive.isBusy())
                    {
                        currentState = State.SAFE2;
                        drive.followTrajectoryAsync(safe2Traj);
                    }
                    break;
                }
                case SAFE2:
                {
                    if(!drive.isBusy())
                    {
                        currentState = State.YELLOW_PIXEL;
                        drive.followTrajectoryAsync(yellowPixelMidTraj);
                        armController.goMid();
                        liftController.goMid();;
                        jointController.goToMid();
                        babi = false;
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
                                    //.lineToConstantHeading(new Vector2d(55, 5))
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
