package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw implements Action {
    private final Servo clawServo;
    public Claw(HardwareMap hardwareMap){
        clawServo = hardwareMap.servo.get("clawServo");
    }

    //    Creating the Telemetry
    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    //    Singleton thingie, making sure there is only one instance of clawMotor
    private static Claw instance = null;
    public static Claw getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Claw(hardwareMap);
        }
        return instance;
    }

    public void resetGrab() {
        clawServo.resetDeviceConfigurationForOpMode();
    }

    public void grabPiece() {
        clawServo.setPosition(0);
    }

    public Action releasePiece() {
        clawServo.setPosition(.5);
        return null;
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }
}
