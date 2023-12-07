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
    @Override
    public void runOpMode() throws InterruptedException {
        //DistanceSensor Colores = hardwareMap.get(DistanceSensor.class, "Color");//

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        double DISTANCE = 60; // in

        waitForStart();
        Random(40, 0);

        //double distance = Colores.getDistance(DistanceUnit.CM);//


        /*if(distance<2.1){
            telemetry.addData("Imhere", distance);
        }
        else{
            Random(0, 90);
            if(distance<2.1){
                telemetry.addData("Imthere", distance);

            }
            else{
                Random(0, 180);
                telemetry.addData("Imzone", distance);
            }
        }*/

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