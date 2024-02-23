package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Autonomous22311REDTEAM extends LinearOpMode {
    ColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Arm = hardwareMap.dcMotor.get("armMotor");

        Servo Gripper = hardwareMap.get(Servo.class, "Grip");
        Servo GripperBack = hardwareMap.get(Servo.class, "Grip1");
        double DISTANCE = 40;
        double ANGLE = -90;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory fwdtrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory starttrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(8)
                .build();

        Trajectory bwdtrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(-DISTANCE/8)
                .build();
        waitForStart();

        if (isStopRequested()) return;
        Gripper.setPosition(0.1);
        GripperBack.setPosition(0.1); //Grab Pixel
//Grab Pixel
        sleep(2000);
        drive.followTrajectory(starttrajectory); // move forward a little
        drive.turn(Math.toRadians(ANGLE));//Turn
        drive.followTrajectory(fwdtrajectory);//move to landing spot
        sleep(2000);
        Gripper.setPosition(1);
        GripperBack.setPosition(1);
        sleep(2000);
        drive.followTrajectory(bwdtrajectory);
        while (!isStopRequested() && opModeIsActive());
    }
}