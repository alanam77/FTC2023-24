package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        while(opModeInInit()){

        }
        while(opModeIsActive()){
            double LeftPower = -gamepad1.left_stick_y;
            double RightPower = -gamepad1.right_stick_y;
            FL.setPower(LeftPower);
            BL.setPower(LeftPower);
            FR.setPower(RightPower);
            BR.setPower(RightPower);

            if(gamepad2.left_bumper){
                claw.setPosition(0);
            }
            else if(gamepad2.right_bumper){
                claw.setPosition(1);
            }
        }

    }
}
