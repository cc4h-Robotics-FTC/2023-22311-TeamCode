package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Autonomous22311 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double DISTANCE = 60; // in

        waitForStart();

        Random(125,-90);
    }

    public double Random(double Xcoord, double ANGLE) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(Xcoord)
                .build();
        drive.followTrajectory(trajectory);
        drive.turn(Math.toRadians(ANGLE));
        return Xcoord;












    }

}