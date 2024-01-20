package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Autonomous22311 extends LinearOpMode {
    ColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo Gripper = hardwareMap.get(Servo.class, "Grip");
        double DISTANCE = 40;
        double ANGLE = 110;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory fwdtrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory bwdtrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(-DISTANCE/10)
                .build();
        waitForStart();

        if (isStopRequested()) return;
        Gripper.setPosition(0.01); //Grab Pixel
        drive.turn(Math.toRadians(ANGLE));//Turn
        drive.followTrajectory(fwdtrajectory);//move to landing spot
        Gripper.setPosition(1.);
        drive.followTrajectory(bwdtrajectory);
        while (!isStopRequested() && opModeIsActive());
    }
}