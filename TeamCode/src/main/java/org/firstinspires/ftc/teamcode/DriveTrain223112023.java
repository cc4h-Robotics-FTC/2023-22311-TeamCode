package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveTrain223112023 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //storing gamepad state for button press
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        boolean LaunchTrue = true;
//        Servo ClawHangServo = hardwareMap.get(Servo.class, "Hanger");
        Servo Launcher = hardwareMap.get(Servo.class, "FLS");
        Servo GripperRight = hardwareMap.get(CRServo.class, "Grip");
        Servo GripperLeft = hardwareMap.get(CRServo.class, "Grip1");
        Launcher.setDirection(Servo.Direction.REVERSE);
        boolean Gripper1 = true;
        boolean Gripper2 = true;
//        Servo Fling = hardwareMap.get(Servo.class, "launcher");
//        Servo grab = hardwareMap.get(Servo.class,"grab");
//
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor Arm = hardwareMap.dcMotor.get("armMotor");
        DcMotor ArmRotator = hardwareMap.dcMotor.get("armRotateMotor");

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

            // save gamepad state for future button press use
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

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
//                else if(LunchTrue = false){
//                    FrontLeftServo.setPosition(0.0);
//            }
//
//
//            }

            if (currentGamepad2.y && !previousGamepad2.y) {
                Launcher.setPosition(0.0);
                time = getRuntime();
                Launcher.setPosition(0.0);
                if (getRuntime() - time > 2000) {
                    Launcher.setPosition(1.0);
                }
            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                GripperLeft.setPower(0);
                GripperRight.setPower(1);
            }

            else if (currentGamepad2.right_bumper && !previousGamepad2.left_bumper) {
                GripperLeft.setPower(1);
                GripperRight.setPower(0);
            }

            else {
                GripperLeft.setPower(0.5);
                GripperRight.setPower(0.5);
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            telemetry.addData("GripperLeft", GripperLeft.getPosition());
            telemetry.addData("GripperRight", GripperRight.getPosition());

            double armZ = gamepad2.right_stick_y;
            double arm_move = gamepad2.left_stick_y;
            int current_pos = Arm.getCurrentPosition();
//
//
//            // claw behavior
//            if(current_pos < 190) { // arm is at the front of the robot
//                if (currentGamepad2.left_trigger > 0) {
//                    ArmRotator.setPower(0.8);
//                } else if (currentGamepad2.right_trigger > 0) {
//                    ArmRotator.setPower(-0.3);
//                } else {
//                    if (ArmRotator.getCurrentPosition() < -260) {
//                        ArmRotator.setPower(0.05);
//                    }
////                    else {
////                        ArmRotator.setPower(0.65);
////                    }`m+
//                }
//            } else if(current_pos>190 && current_pos < 250) { // arm is at the back of the robot
//                if (currentGamepad2.left_trigger > 0) {
//                    ArmRotator.setPower(0.7);
//                } else if (currentGamepad2.right_trigger > 0) {
//                    ArmRotator.setPower(-0.7);
//                } else {
//                    ArmRotator.setPower(-0.05);
//                }
//            } else if (current_pos >= 250) { // lower the claw
//                if (currentGamepad2.left_trigger > 0) {
//                    ArmRotator.setPower(0.2);
//                } else if (currentGamepad2.right_trigger > 0) {
//                    ArmRotator.setPower(-0.8);
//                } else {
//                    ArmRotator.setPower(-0.05);
//                }
//            } else {
//                ArmRotator.setPower(0.1); // just a bit of power to prevent dead falling, dead falling: sudden power cut
//            }


            // claw behavior
            int Stability = ArmRotator.getCurrentPosition();
            ArmRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            if (current_pos < 190) { // arm is at the front of the robot
//                if (currentGamepad2.left_trigger > 0) {
//                    ArmRotator.setPower(gamepad2.left_trigger/1.5);
//                } else if (currentGamepad2.right_trigger > 0) {
//                    ArmRotator.setPower(-gamepad2.right_trigger/1.5);
//                }
//            } else if (current_pos > 190 && current_pos < 250) { // arm is at the back of the robot
//                if (currentGamepad2.left_trigger > 0) {
//                    ArmRotator.setPower(gamepad2.left_trigger/1.5);
//                } else if (currentGamepad2.right_trigger > 0) {
//                    ArmRotator.setPower(-gamepad2.right_trigger/1.5);
//                }
//            } else if (current_pos >= 250) { // lower the claw
//                if (currentGamepad2.left_trigger > 0) {
//                    ArmRotator.setPower(gamepad2.left_trigger/1.5);
//                } else if (currentGamepad2.right_trigger > 0) {
//                    ArmRotator.setPower(-gamepad2.right_trigger/1.5);
//                }
            if (gamepad2.left_trigger > 0) {
                ArmRotator.setPower(gamepad2.left_trigger / 1.5);
            } else if (gamepad2.right_trigger > 0) {
                ArmRotator.setPower(-gamepad2.right_trigger/1.8);
            } else {
                ArmRotator.setPower(0.1); // just a bit of power to prevent dead falling, dead falling: sudden power cut
            }

            if(gamepad2.a){
                ArmRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                ArmRotator.setTargetPosition(Stability+10);
            }
            if(gamepad2.b){
                ArmRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            telemetry.addData("Left Trigger", gamepad2.left_trigger);

            // todo: when at a vertical position; it should break and then if the user continues to press down it breaks
            // todo: find the -340 and -180;
            // i luv 69


            int vertical_pos = 255;
//            if (arm_move < 0) { // raise arm
//                if (current_pos < 190) {
//                    Arm.setPower(0.75); // raise arm steadily with great power
//                } else if (current_pos >= 190) {
//                    Arm.setPower(0.15); // we are past vertical piont, we don't need too much power
//                }
//            } else{
//                Arm.setPower(0.1);// to go down code; makes it such that when the robot goes down, it isn't very fast
//            }
//
//
//
//            if (arm_move > 0) { // lower arm
//                if (current_pos > 190) {
//                    Arm.setPower(-0.65);
//                } else {
//                    Arm.setPower(-0.05); //
//                }
//            }
//
//            if (current_pos < 120 && arm_move == 0) { // default arm behavior
//                    Arm.setPower(0.1); // to stop the arm from free falling
//            }

        if (arm_move < 0) { // raise arm
            Arm.setPower(-gamepad2.left_stick_y);

//            if (current_pos < 190) {
//                Arm.setPower(Math.max(gamepad2.left_stick_y, 0.7)); // raise arm steadily with great power
//            } else if (current_pos >= 190) {
//                Arm.setPower(Math.min(gamepad2.left_stick_y, 0.6)); // we are past vertical piont, we don't need too much power
//            }
//            else {
//                Arm.setPower(-gamepad2.left_stick_y/10);// to go down code; makes it such that when the robot goes down, it isn't very fast
//            }
        } else if (current_pos > 0) {
            Arm.setPower(0.1); // prevent dead falling+
        }


        if (arm_move > 0) { // lower arm
            Arm.setPower(-gamepad2.left_stick_y);
//            Arm.setPower(-Math.min(gamepad2.left_stick_y, 0.6));
//            if (current_pos > 190) {
//                Arm.setPower(-0gamepad2.left_stick_y);
//            } else {
//                Arm.setPower(-gamepad2.left_stick_y); //
//            }
        }

//        if (current_pos < 120 && arm_move == 0) { // default arm behavior
//            Arm.setPower(-gamepad2.left_stick_y); // to stop the arm from free falling
//        }

            telemetry.addData("Arm Pow", Arm.getPower());
            telemetry.addData("Arm Pos", Arm.getCurrentPosition());

            telemetry.addData("ArmClawPow Power", ArmRotator.getPower());
            telemetry.addData("ArmClawPos", ArmRotator.getCurrentPosition());

            telemetry.addData("LeftStick", currentGamepad2.left_stick_y);
            telemetry.update();


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
//            double Armpowerrotator = armZ/3;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
//            ArmRotator.setPower(Armpowerrotator);


        }
    }
}
