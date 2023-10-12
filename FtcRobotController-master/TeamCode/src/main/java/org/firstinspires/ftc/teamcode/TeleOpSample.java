package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "teleop")
public class TeleOpSample extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftRear;
    private DcMotor RightRear;
    private DcMotor arm;
    private DcMotor lift;
    private Servo s1;
    private CRServo s2;
    private Servo s3;
    private Servo s4;
    @Override
    public void runOpMode() throws InterruptedException {
        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftRear = hardwareMap.get(DcMotor.class, "left_rear");
        RightRear = hardwareMap.get(DcMotor.class, "right_rear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(DcMotor.class, "arm");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.FORWARD);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeInInit()){

        }
        double position = 0;
        //MotorController(arm, (int)position);
        while(opModeIsActive()){
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            LeftFront.setPower(v1);
            RightFront.setPower(v2);
            LeftRear.setPower(v3);
            RightRear.setPower(v4);

            lift.setPower(-gamepad1.right_stick_y);

            position += Math.abs(gamepad2.left_stick_y) < 0.1 ? 0 : -gamepad2.left_stick_y * 2;
            if(position < 0){
                position = 0;
            }
            MotorController(arm, (int)position);

            s2.setPower(-gamepad2.right_stick_y);

            if(gamepad1.left_bumper){
                s3.setPosition(0.1);
            }
            else if(gamepad1.right_bumper){
                s3.setPosition(0.2);
            }
            if (gamepad2.x){
                s1.setPosition(0.5);
            }
            else if(gamepad2.b){
                s1.setPosition(0.9);
            }
            if(gamepad2.dpad_up){
                s4.setPosition(0.4);
            }
            else{
                s4.setPosition(0);
            }
            telemetry.addData("Arm Position: ", (int)position);
            telemetry.update();
        }
    }
    public void MotorController(DcMotor arm, int pos){
        double speed = 1;
        arm.setTargetPosition(pos);
        arm.setPower(speed);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
