package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drawing;

//importing all the subsystems
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Elevator;

import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Rack;
import org.firstinspires.ftc.teamcode.ClawPivot;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public class enabling extends LinearOpMode {

    //elevatorMotor;
    Elevator elevator;
    Claw claw;
    ClawPivot clawPivot;
    Arm arm;
    Rack rack;

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //constants and objects
        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        int[] validIDs = {3, 4};
        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        elevator = new Elevator(hardwareMap);
        claw = new Claw(hardwareMap);
        claw.resetGrab();
        arm = new Arm(hardwareMap);
        rack = new Rack(hardwareMap);
        clawPivot = new ClawPivot(hardwareMap);
        clawPivot.resetClawPivot();
        double clawPivotPosition = 0;

        arm.retractArm();
        claw.releasePiece();

        // rising edge
        boolean clawBoom = false;
        boolean armBoom = false;

        final double headingKP = 0.023;

        telemetry.setMsTransmissionInterval(11);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.start();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            waitForStart();

            if (isStopRequested()) return;

            boolean prevRightBumper = gamepad1.right_bumper;
            boolean prevLeftBumper = gamepad1.left_bumper;

            while (opModeIsActive()) {

//                arm.retractArm();
//                claw.releasePiece();

                // Convert the robot's current heading (in radians) to degrees.
                double headingMeasurement = Math.toDegrees(drive.pose.heading.toDouble());

                // Set the desired rotational velocity based on the right stick's x-axis input (clockwise is negative).
                double headingVel = -gamepad1.right_stick_x;
                double headingSetpoint = 0;
                boolean closedLoop = gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up;
                if (gamepad1.back) {
                    drive.pose = new Pose2d(drive.pose.position, 0);
                }
                if (gamepad1.dpad_up) {
                    headingSetpoint = 0;
                }
                if (gamepad1.dpad_right) {
                    headingSetpoint = -90;
                }
                if (gamepad1.dpad_left) {
                    headingSetpoint = 90;
                }
                if (gamepad1.dpad_down) {
                    headingSetpoint = 180;
                }

                // If closed-loop control is enabled, calculate the error between the desired and current heading.
                if (closedLoop) {
                    // Define the error bounds within [-180, 180] degrees for the heading.
                    double errorBound = (180.0 - (-180)) / 2.0;
                    double headingError = inputModulus(headingSetpoint - headingMeasurement, -errorBound, errorBound);
                    // Adjust the rotational velocity based on the heading error and a proportional gain (headingKP).
                    headingVel = headingError * headingKP;
                }

                //Field Centric Drive
                double flippedHeading = drive.pose.heading.inverse().toDouble();

                Vector2d driveVectorNonRotated = new Vector2d(
                        -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * -gamepad1.left_stick_x * -gamepad1.left_stick_x
                );
                if (arm.getArmPosition() == 0 || elevator.getPosition() > 18) {
                    driveVectorNonRotated = new Vector2d(
                            -gamepad1.left_stick_y * -gamepad1.left_stick_y * -gamepad1.left_stick_y * 0.25,
                            -gamepad1.left_stick_x * -gamepad1.left_stick_x * -gamepad1.left_stick_x * 0.25
                    );
                }

                Vector2d driveVectorRotated = new Vector2d(
                        driveVectorNonRotated.x * Math.cos(flippedHeading) - driveVectorNonRotated.y * Math.sin(flippedHeading),
                        driveVectorNonRotated.x * Math.sin(flippedHeading) + driveVectorNonRotated.y * Math.cos(flippedHeading)
                );

                drive.setDrivePowers(new PoseVelocity2d(
                        driveVectorRotated,
                        headingVel
                ));
                drive.updatePoseEstimate();

                // Elevator
                // High Basket
                if (gamepad1.y) {
                    elevator.runToPosition(38);
                }
                // Low Basket
                if (gamepad1.b) {
                    elevator.update();
                    elevator.runToPosition(18);
                }
                if (gamepad1.a) {
                    elevator.runToPosition(0);
                }
                if (gamepad1.start) {
                    elevator.reZero();
                }
                // Making the elevator go up or down
                if (gamepad1.right_trigger > 0) {
                    elevator.setElevatorPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0) {
                    elevator.setElevatorPower(-gamepad1.left_trigger);
                }
                elevator.update();

                //claw stuff
                boolean rightBumperPressed = !prevRightBumper && gamepad1.right_bumper;
                prevRightBumper = gamepad1.right_bumper;

                if (rightBumperPressed) {
                    if (clawBoom) {
                        clawBoom = false;
                    } else {
                        clawBoom = true;
                    }
                    if (clawBoom) {
                        claw.grabPiece();
                    } else {
                        claw.releasePiece();
                    }
                }

                if (gamepad2.right_trigger > 0) {
                    rack.moveRack(gamepad2.right_trigger);
                }
                if (gamepad2.left_trigger > 0) {
                    rack.moveRack(-gamepad2.left_trigger);
                }

                if (gamepad2.a) {
                    clawPivotPosition = .5;
                    clawPivot.pivotClawSet(clawPivotPosition);
                }

                //arm control
                boolean leftBumperPressed = !prevLeftBumper && gamepad1.left_bumper;
                prevLeftBumper = gamepad1.left_bumper;

                if (leftBumperPressed) {
                    if (armBoom) {
                        armBoom = false;
                    } else {
                        armBoom = true;
                    }
                    if (armBoom) {
                        arm.extendArm();
                    } else {
                        arm.retractArm();
                    }
                }

                /*
                private boolean shouldUseMegatag() {

                }

                private boolean shouldUseMegatag2() {

                }
                */

                limelight.updateRobotOrientation(headingMeasurement);
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose_MT2();
                        telemetry.addData("llpose", botpose.toString());
                    }
                    else {
                        telemetry.addData("llpose", "NONE");
                    }
                }

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", headingMeasurement);
                telemetry.addData("Elevator Pos (in)", elevator.getPosition());
                telemetry.addData("Claw Servo Position:", claw.getClawPosition());
                telemetry.addData("Arm Servo Position:", arm.getArmPosition());
                telemetry.addData("Rack Motor Position", rack.getRackPosition());
                telemetry.addData("Claw Pivot Servo Position,", clawPivot.getClawPivotPosition());

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();

            }
        }
    }
}
