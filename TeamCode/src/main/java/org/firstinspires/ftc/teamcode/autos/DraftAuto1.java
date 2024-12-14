package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Elevator;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//subsystems
import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Rack;
import org.firstinspires.ftc.teamcode.ClawPivot;

@Autonomous (name = "DraftAuto1")
public class DraftAuto1 extends LinearOpMode {
    Elevator elevator;
    Claw claw;
    ClawPivot clawPivot;
    Arm arm;
    Rack rack;

    @Override
    public void runOpMode() throws InterruptedException {
        //mapping the hardware
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        elevator = new Elevator(hardwareMap);
        claw = new Claw(hardwareMap);
        claw.resetGrab();
        arm = new Arm(hardwareMap);
        rack = new Rack(hardwareMap);
        clawPivot = new ClawPivot(hardwareMap);
        clawPivot.resetClawPivot();



        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(12)
                        .stopAndAdd(elevator.runToPosition(18))
                        .stopAndAdd(claw.releasePiece())
                        .lineToX(0)
                       // .splineToLinearHeading()
                        .build());

    }
}
