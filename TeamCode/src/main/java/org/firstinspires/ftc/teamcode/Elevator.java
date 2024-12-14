package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//Constructor
public class Elevator implements Action {
    private final DcMotor elevatorMotor;
    double elevatorScale = 1.0/100.0;
    double currentPosInches;
    double setpoint = 0;
    double lastError = 0;
    double elevatorOffset = 0;
    double elevatorPosition;


    public Elevator(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.dcMotor.get("elevatorMotor");
    }

//    Creating the Telemetry
    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

//    isBusy checks if the motor is doing anything, like if it's running to position for example
    private boolean busy;
    public boolean isBusy() {
        return this.busy;
    }

//    Singleton thingie, making sure there is only one instance of elevatorMotor
    private static Elevator instance = null;
    public static Elevator getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Elevator(hardwareMap);
        }
        return instance;
    }

    private boolean runningToPosition;
    private double positionCmdInches;

    final double kP = 1.0 / 2.0;
    final double kD = 0;

    public void reZero(){
        elevatorOffset = elevatorMotor.getCurrentPosition();
    }

    public double getPosition() {
        elevatorPosition = elevatorScale * (elevatorMotor.getCurrentPosition() - elevatorOffset);
        return elevatorPosition;
    }

    public Action runToPosition(double positionInches) {
        positionCmdInches = positionInches;
        runningToPosition = true;
        busy = true;
        elevatorMotor.setTargetPosition((int)(positionInches*100));

        setpoint = positionInches * elevatorScale;
        double error = setpoint - elevatorPosition;
        double changeInError = error - lastError;
        double elevatorPower = kP * error + kD * changeInError;
        lastError = error;

        elevatorMotor.setPower(elevatorPower);
        return null;
    }


    private double desiredPower;
    public void setElevatorPower(double power) {
        elevatorMotor.setPower(power);
        this.desiredPower = power;
        runningToPosition = false;
    }

    private double lastPosition = 0;

    public void update() {
//        Check if the elevator is currently moving to a target position
        double currentPosInches = (double) elevatorMotor.getCurrentPosition() * elevatorScale;
        this.lastPosition = currentPosInches;

        if (runningToPosition) {
//            Put logic for running to position with PID
//            Get the current position of the elevator motor in inches
//            Difference between the target position and the current position (error)
            double error = positionCmdInches - currentPosInches;

            if (telemetry != null) {
                telemetry.addData("Elevator Error Inches", error);
            }

//            Change of error (how much the error is changing)
            double changeInError = error - lastError;

//            kP: Proportional constant (adjusts based on the distance from the target)
//            kD: Derivative constant (adjusts based on the rate of change of the error)
            double elevatorPower = (kP * error) + (kD * changeInError);
            elevatorMotor.setPower(elevatorPower);
            lastError = error;

//            if it's too close to zero it stops in fear of exploding
            if (Math.abs(error) < 2.5 || (positionCmdInches == 1 && currentPosInches < 3)) {
                runningToPosition = false;
                busy = false;
                elevatorMotor.setPower(0);
            }
        } else {
            elevatorMotor.setPower(this.desiredPower);
        }

//        if nothing is being printed to telemetry, update the telemetry
        if (telemetry != null) {
            telemetry.update();
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }

}

