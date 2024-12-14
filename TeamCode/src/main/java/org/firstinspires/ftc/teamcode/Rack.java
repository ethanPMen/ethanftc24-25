package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Rack {
    private final DcMotor rackMotor;

    public Rack(HardwareMap hardwareMap) {
        rackMotor = hardwareMap.dcMotor.get("rackMotor");
    }

    public void moveRack(double power) {
        rackMotor.setPower(power);
    }

    public int getRackPosition() {
        return rackMotor.getCurrentPosition();
    }


}
