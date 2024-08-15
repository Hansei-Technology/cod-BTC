package org.firstinspires.ftc.teamcode.NEDRobot.Commands;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.BucketPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.CapacPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.PitchPosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Base_Commands.WristOuttakePosCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Obot;

public class DepositCommand extends SequentialCommandGroup {
    public DepositCommand(Obot obot){
        super(
                new SequentialCommandGroup(
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.IN),
                        new WaitCommand(700),
                        new InstantCommand(() -> obot.linkage.setPosition(0.65)),
                        new WaitCommand(700),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.SEMIOUT),
                        new WaitCommand(700),
                        new BucketPosCommand(obot, LiftSubsystem.BucketState.LOW),
                        new WaitCommand(700),
                        new WristOuttakePosCommand(obot, LiftSubsystem.WristState.HORIZONTAL_DEPOSIT),
                        new WaitCommand(700),
                        new PitchPosCommand(obot, LiftSubsystem.PitchState.LOW),
                        new CapacPosCommand(obot, IntakeSubsystem.CapacState.CLOSED)
                )
        );
    }
}
