package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveTrain223112023 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean LaunchTrue = true;
        Servo ClawHangServo = hardwareMap.get(Servo.class, "Hanger");
        Servo FrontLeftServo = hardwareMap.get(Servo.class, "FLS");
        Servo Gripper = hardwareMap.get(Servo.class, "Grip");
        FrontLeftServo.setDirection(Servo.Direction.REVERSE);
//
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor= hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.left_bumper){
                if(LaunchTrue = true) {
                    time = getRuntime();
                    FrontLeftServo.setPosition(1.0);
                    if (getRuntime() - time > 2000) {
                        FrontLeftServo.setPosition(0.0);

                    }
                    LaunchTrue = false;
                }

                else if(LaunchTrue = false){
                    FrontLeftServo.setPosition(0.0);
            }


            }

            if(gamepad1.a){
                ClawHangServo.setPosition(1.0);
            }

            if(gamepad1.b){
                ClawHangServo.setPosition(0.0);
            }

            if(gamepad2.a){
                Gripper.setPosition(1.0);
            }

            if(gamepad2.b){
                Gripper.setPosition(0.25);

            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


        }
    }
}
