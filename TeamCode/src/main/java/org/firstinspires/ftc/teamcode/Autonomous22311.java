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
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        boolean Tunk = true;

        double DISTANCE = 60; // in
        waitForStart();
        Random(26, 0);
        int colorVal = readColor();
        if ((colorVal == 1) || (colorVal == 2)) {
            Random(5, 0);

        } else {
            Random(0.1, 150);
            if ((colorVal == 1) || (colorVal == 2)) {
                Random(5, 0);

            } else {
                Random(0.1, 210);
                Random(5, 0);

            }
        }
    }

//
//

    

    public double Random(double Xcoord, double ANGLE) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(Xcoord)
                .build();
        drive.followTrajectory(trajectory);
        drive.turn(Math.toRadians(ANGLE));
        return Xcoord;












    }

    public int readColor() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();


        int largerNumber = Math.max(red, blue);
        int LargestNumberChoice = Math.max(green, largerNumber);

        if (LargestNumberChoice == red) {
            return 1;
        }

        else if (LargestNumberChoice == blue){
            return 2;
        }

        else if (LargestNumberChoice == green){
            return 3;
        }


        return 0;
    }
}