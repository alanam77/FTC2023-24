package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class EmptySample extends LinearOpMode {
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private Servo claw;
    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "Front Left");
        FR = hardwareMap.get(DcMotor.class, "Front Right");
        BL = hardwareMap.get(DcMotor.class, "Back Left");
        BR = hardwareMap.get(DcMotor.class, "Back Right");
        claw = hardwareMap.get(Servo.class, "claw");
        while(opModeInInit()){

        }
        while(opModeIsActive()){

        }

    }
}
