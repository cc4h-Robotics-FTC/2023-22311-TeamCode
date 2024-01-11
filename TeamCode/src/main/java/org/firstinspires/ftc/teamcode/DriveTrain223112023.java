package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveTrain223112023 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean LaunchTrue = true;
//        Servo ClawHangServo = hardwareMap.get(Servo.class, "Hanger");
        Servo FrontLeftServo = hardwareMap.get(Servo.class, "FLS");
        Servo Gripper = hardwareMap.get(Servo.class, "Grip");
        FrontLeftServo.setDirection(Servo.Direction.REVERSE);

//        Servo Fling = hardwareMap.get(Servo.class, "launcher");
//        Servo grab = hardwareMap.get(Servo.class,"grab");
//
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor= hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor Arm = hardwareMap.dcMotor.get("armMotor");

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if (isStopRequested()) return;
        int targetPos = -10;
        while (opModeIsActive()) {

//            if(gamepad1.left_bumper){
//                if(LaunchTrue = true) {
//                    time = getRuntime();
//                    FrontLeftServo.setPosition(1.0);
//                    if (getRuntime() - time > 2000) {
//                        FrontLeftServo.setPosition(0.0);
//
//                    }
//                    LaunchTrue = false;
//                }
//
//                else if(LaunchTrue = false){
//                    FrontLeftServo.setPosition(0.0);
//            }
//
//
//            }

            if (gamepad2.y) {
                FrontLeftServo.setPosition(0.0);
                time = getRuntime();
                FrontLeftServo.setPosition(0.0);
                if (getRuntime() - time > 2000) {
                    FrontLeftServo.setPosition(1.0);
                }
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
//            double arm_down = gamepad2.left_trigger;
//            double arm_up = gamepad2.right_trigger;

//            if (arm_down>0){
//                double arm_power = Arm.getPower(); // get the power and adjust them based on the Arm Power
//                telemetry.addData("Arm power", arm_power);
//                Arm.setPower(0.3);
//            }else if (arm_up>0){
//                Arm.setPower(-0.3);
//                double arm_power = Arm.getPower(); // get the power and adjust them based on the Arm Power
//                telemetry.addData("Arm power", arm_power);
//            } else {
//                Arm.setPower(-0.2);
//                double arm_power = Arm.getPower(); // get the power and adjust them based on the Arm Power
//                telemetry.addData("Arm power", arm_power);
//            }


//            int vertical_pos = -210;
//            int lowest_pos = -40;
//            double handPower = 0.;
//
//            final double maxHandPower = 1.0;
//            final double powerIncrement = 0.55;
//            final double powerDecrease = 0.7;


            double arm_move = gamepad2.left_stick_y;
//            double arm_up = gamepad2.left_stick_y;
//            if (arm_up > 0) {
//                int position = Arm.getCurrentPosition();
//                if (position<vertical_pos){
////                    handPower -= powerIncrement/3 * arm_up;
//                    position -= powerIncrement;
//                    Arm.setTargetPosition(position);
//
//                } else {
//                    handPower -= powerIncrement * arm_up;
//                }
//                if (handPower < -1) {
//                    handPower = -1;
//                }
//                Arm.setPower(handPower);
//            } else if (arm_down < 0) {
//                handPower += -1*powerDecrease * arm_down;
//                if (handPower > maxHandPower) {
//                    handPower = maxHandPower;
//                }
//                Arm.setPower(handPower);
//            } else {
//                int position = Arm.getCurrentPosition();
//                Arm.setTargetPosition(position);
//                Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            }
//            int current_pos = Arm.getCurrentPosition();
//            if (current_pos < lowest_pos) {
//                Arm.setTargetPosition(lowest_pos);
//            }


//            final double pos_change = 0.55;
////            final double powerDecrease = 0.7;
//
//            if (arm_up > 0) {
//                int position = Arm.getCurrentPosition();
//                Arm.setTargetPosition(position + pos_change);
//            } else if (arm_down < 0) {
//                handPower += -1*powerDecrease * arm_down;
//                if (handPower > maxHandPower) {
//                    handPower = maxHandPower;
//                }
//                Arm.setPower(handPower);
//            } else {
//                int position = Arm.getCurrentPosition();
//                Arm.setTargetPosition(position);
//                Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            }
//            int current_pos = Arm.getCurrentPosition();
//            if (current_pos < lowest_pos) {
//                Arm.setTargetPosition(lowest_pos);
//            }


            // todo: when at a vertical position; it should break and then if the user continues to press down it breaks
            // todo: find the -340 and -180;
            int vertical_pos = -270;
            int current_pos = Arm.getCurrentPosition();
            if (arm_move < 0) {
                if (current_pos < -250) {
                    Arm.setPower(0.7);
                } else {
                    Arm.setPower(0.1);
                }

            } else if (arm_move > 0) {
                if (current_pos > -180) {
                    Arm.setPower(-0.7);
                } else {
                    Arm.setPower(-0.1);
                }


            } else {
                if (current_pos < -10 && current_pos > -180) {
                    Arm.setPower(-0.2);
                } else if (current_pos < -320) {
                    Arm.setPower(0.2);
                } else {
                    Arm.setPower(0);
                }

            }

//            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Arm Pwer", Arm.getPower());
            telemetry.addData("Arm Pos", Arm.getCurrentPosition());
            telemetry.update();

            if (gamepad2.x) {
                Arm.setPower(0.0);
                Gripper.setPosition(0.01);
                sleep(2000);
                Arm.setPower(-0.2);
            }

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
