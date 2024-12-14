package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private final Servo armServo;
    public Arm(HardwareMap hardwareMap){
        armServo = hardwareMap.servo.get("armServo");
        armServo.setDirection(Servo.Direction.REVERSE);
    }

    //    Creating the Telemetry
    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    //    Singleton thingie, making sure there is only one instance of clawMotor
    private static Arm instance = null;
    public static Arm getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Arm(hardwareMap);
        }
        return instance;
    }

    public void resetArm() {
        armServo.resetDeviceConfigurationForOpMode();
    }

    public void retractArm() {
        armServo.setPosition(.5);
    }

    public void extendArm() {
        armServo.setPosition(0);
    }

    public double getArmPosition() {
        return armServo.getPosition();
    }
}
