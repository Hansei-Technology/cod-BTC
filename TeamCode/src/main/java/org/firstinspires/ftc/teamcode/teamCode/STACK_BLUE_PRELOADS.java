//package org.firstinspires.ftc.teamcode.teamCode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
//
//@Autonomous
//@Config
//public class STACK_BLUE_PRELOADS extends LinearOpMode {
//
//    enum State {
//        PRELOAD_LINE,
//        YELLOW_PIXEL,
//        PARK,
//        IDLE,
//        SAFE1,
//        SAFE2
//    }
//
//    State currentState = State.IDLE;
//    Pose2d startPose = new Pose2d( -35.5, 62.5, Math.toRadians(-90));
//    Pose2d SAFE1POS = new Pose2d(-45, -3, Math.toRadians(0));
//    Pose2d SAFE2POS = new Pose2d(15, -3, Math.toRadians(0));
//    ElapsedTime timerBabi;
//    Boolean babi;
//
//    enum random {
//        LEFT,
//        RIGHT,
//        MID
//    }
//    random result = random.MID;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        timerBabi = new ElapsedTime();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(startPose);
//        LiftController liftController = new LiftController(hardwareMap);
//        ClawController2Servo clawController = new ClawController2Servo(hardwareMap);
//        JointController2Servo jointController = new JointController2Servo(hardwareMap);
//        ArmController armController  =  new ArmController(hardwareMap);
//
//        Trajectory preloadLineRightTraj = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(-110))) //right
//                .build();
//        Trajectory preloadLineMidTraj = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-45, 0.5, Math.toRadians(45))) //mid
//                .build();
//        Trajectory preloadLineLeftTraj = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-38, 54, Math.toRadians(-56))) //left
//                .build();
//        Trajectory yellowPixelRightTraj = drive.trajectoryBuilder(SAFE2POS) // right
//                .lineToLinearHeading(new Pose2d(46.5, 20, Math.toRadians(-8)))
//                .build();
//        Trajectory yellowPixelMidTraj = drive.trajectoryBuilder(SAFE2POS) //mid
//                .lineToLinearHeading(new Pose2d(45, 25.5, Math.toRadians(0)))
//                .build();
//        Trajectory yellowPixelLeftTraj = drive.trajectoryBuilder(SAFE2POS) //left
//                .lineToLinearHeading(new Pose2d(45.5, 33, Math.toRadians(0)))
//                .build();
//        Trajectory safe2Traj = drive.trajectoryBuilder(SAFE1POS)
//                .lineToLinearHeading(SAFE2POS)
//                .build();
//
//
//        ElapsedTime timeLeft = new ElapsedTime();
//
//        jointController.goToMid();
//        clawController.toggleLeft();
//        clawController.toggleRight();
//
//        armController.goToPoz(700);
//        liftController.goTOPos(-50);
//
//
//        while(opModeInInit()) {
//            armController.update();
//            liftController.update();
//            if(armController.currentPos > 500) {
//                jointController.goToPoz(0.87);
//                armController.goToPoz(550);
//            }
//        }
//
//        liftController.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftController.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//
//        timeLeft.reset();
//        if (isStopRequested()) return;
//        currentState = State.PRELOAD_LINE;
//        drive.followTrajectoryAsync(preloadLineMidTraj);
//        babi = false;
//
//        while (opModeIsActive() && !isStopRequested())
//        {
//            switch (currentState)
//            {
//                case PRELOAD_LINE:
//                {
//                    armController.goDown();
//                    if(result == random.MID) liftController.goTOPos(250);
//                    else liftController.goTOPos(liftController.MaxPoz);
//                    jointController.goToDown();
//                    if (!drive.isBusy())
//                    {
//                        if(!babi) timerBabi.reset();
//                        clawController.openRight();
//                        babi = true;
//                        if(timerBabi.milliseconds() > 200) {
//                            currentState = State.SAFE1;
//                            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(SAFE1POS)
//                                    .build();
//                            drive.followTrajectoryAsync(traj);
//                            liftController.goTOPos(0);
//                            armController.goMid();
//                        }
//                    }
//                    break;
//                }
//                case SAFE1:
//                {
//                    if(!drive.isBusy())
//                    {
//                        currentState = State.SAFE2;
//                        drive.followTrajectoryAsync(safe2Traj);
//                    }
//                    break;
//                }
//                case SAFE2:
//                {
//                    if(!drive.isBusy())
//                    {
//                        currentState = State.YELLOW_PIXEL;
//                        drive.followTrajectoryAsync(yellowPixelMidTraj);
//                        armController.goMid();
//                        liftController.goMid();;
//                        jointController.goToMid();
//                        babi = false;
//                    }
//                    break;
//                }
//                case YELLOW_PIXEL:
//                {
//                    if(!drive.isBusy())
//                    {
//                        if(!babi) timerBabi.reset();
//                        babi = true;
//                        clawController.openLeft();
//                        if(timerBabi.milliseconds() > 200) {
//                            liftController.goDown();
//                            currentState = State.PARK;
//                            Trajectory parkTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToConstantHeading(new Vector2d(41, -2))
//                                    //.lineToConstantHeading(new Vector2d(55, 5))
//                                    .build();
//                            drive.followTrajectoryAsync(parkTraj);
//
//                            liftController.goTOPos(0);
//                        }
//                    }
//                    break;
//                }
//                case PARK:
//                {
//                    if(!drive.isBusy())
//                    {
//                        armController.goMid();
//                        jointController.goToDown();
//                    }
//                    break;
//                }
//                case IDLE:
//                    break;
//            }
//
//
//            liftController.update();
//            armController.update();
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//
//            // Print pose to telemetry
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
////            telemetry.addData("Time Left", timeLeft.seconds());
//            telemetry.update();
//        }
//    }
//}
