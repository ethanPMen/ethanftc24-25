package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawPivot
{
    private final Servo clawPivotServo;

    public ClawPivot(HardwareMap hardwareMap){
        clawPivotServo = hardwareMap.servo.get("clawPivotServo");
    }

    //    Creating the Telemetry
    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private static ClawPivot instance = null;
    public static ClawPivot getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new ClawPivot(hardwareMap);
        }
        return instance;
    }

    public void pivotClawSet(double position) {
        clawPivotServo.setPosition(position);
    }

    public double getClawPivotPosition() {
        return clawPivotServo.getPosition();
    }

    public void resetClawPivot() {
        clawPivotServo.resetDeviceConfigurationForOpMode();
    }
}
