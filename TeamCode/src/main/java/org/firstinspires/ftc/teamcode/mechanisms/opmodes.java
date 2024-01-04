package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public abstract class opmodes extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();

    @Override
    public void init(){
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        telemetry.addData("touch sensor", board.getTouchSensor());
    }
}
