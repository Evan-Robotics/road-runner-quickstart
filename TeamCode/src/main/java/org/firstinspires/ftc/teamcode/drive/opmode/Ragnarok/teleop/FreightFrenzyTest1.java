package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp
public class FreightFrenzyTest1 extends LinearOpMode {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor spinny_thing;
    private DcMotor intake_main;
    private DcMotor intake_2;
    private Servo bucket;


    @Override
    public void runOpMode() {


        front_left = hardwareMap.get(DcMotor.class, "FRONT LEFT");
        front_right = hardwareMap.get(DcMotor.class, "FRONT RIGHT");
        back_left = hardwareMap.get(DcMotor.class, "BACK LEFT");
        back_right = hardwareMap.get(DcMotor.class, "BACK RIGHT");

        spinny_thing = hardwareMap.get(DcMotor.class, "SPINNY THING");
        intake_main = hardwareMap.get(DcMotor.class, "INTAKE MAIN");
        intake_2 = hardwareMap.get(DcMotor.class, "INTAKE 2");

        bucket = hardwareMap.get(Servo.class, "BUCKET");

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        spinny_thing.setDirection(DcMotor.Direction.FORWARD);
        intake_main.setDirection(DcMotor.Direction.REVERSE);
        intake_2.setDirection(DcMotor.Direction.FORWARD);

        bucket.setDirection(Servo.Direction.REVERSE);

        spinny_thing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double front_left_power;
        double front_right_power;
        double back_left_power;
        double back_right_power;

        String run_type = "pov";

        boolean spinny_direction = false;
        boolean spinny_direction_toggler = false;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_left) {
                run_type = "pov";
            } else if(gamepad1.dpad_right) {
                run_type = "tank";
            }

            front_left_power = 0;
            front_right_power = 0;
            back_left_power = 0;
            back_right_power = 0;

            switch(run_type) {
                case "pov": {
                    int strafeDirection =  1; // negative or positive
                    int turnDirection   =  1; // negative or positive
                    double moveSpeed   = -gamepad1.left_stick_y;
                    double turnSpeed   =  gamepad1.right_stick_x * turnDirection;
                    double strafeSpeed =  gamepad1.left_stick_x * strafeDirection;

                    front_left_power = moveSpeed + turnSpeed + strafeSpeed;
                    front_right_power = moveSpeed - turnSpeed - strafeSpeed;
                    back_left_power = moveSpeed + turnSpeed - strafeSpeed;
                    back_right_power = moveSpeed - turnSpeed + strafeSpeed;
                    break;
                }
                case "tank": {
                    front_left_power = -gamepad1.left_stick_y;
                    front_right_power = -gamepad1.right_stick_y;
                    back_left_power = -gamepad1.left_stick_y;
                    back_right_power = -gamepad1.right_stick_y;
                    break;
                }
                default: {
                    telemetry.addData("ERROR", "Run type not found");
                    break;
                }
            }
            double speedChange1;
            if (gamepad1.left_bumper) {
                speedChange1 = 0.5;
            } else if (gamepad1.right_bumper) {
                speedChange1 = 1;
            } else {
                speedChange1 = 0.7;
            }

            front_left_power *= speedChange1;
            front_right_power *= speedChange1;
            back_left_power *= speedChange1;
            back_right_power *= speedChange1;


            front_right.setPower(front_right_power);
            front_left.setPower(front_left_power);
            back_right.setPower(back_right_power);
            back_left.setPower(back_left_power);



            if (spinny_direction_toggler && !gamepad1.y) {
                spinny_direction = !spinny_direction;
            }
            spinny_direction_toggler = gamepad1.y;

            spinny_thing.setPower( spinny_direction ? speedChange1 * gamepad1.left_trigger : -1 * speedChange1 * gamepad1.left_trigger );

            intake_main.setPower(gamepad1.right_trigger * speedChange1);
            intake_2.setPower(gamepad1.right_trigger * speedChange1);

            if (gamepad1.a) {
                bucket.setPosition(1);
            }
            else {
                bucket.setPosition(0);
            }

            telemetry.addData("Status", "Running");
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("Front Left", front_left_power);
            telemetry.addData("Front Right", front_right_power);
            telemetry.addData("Back Left", back_left_power);
            telemetry.addData("Back Right", back_right_power);
            telemetry.addData("Run Type", run_type);
            telemetry.addData("Intake Power", intake_main.getPower());
            telemetry.addData("Bucket Pos", bucket.getPosition());
            telemetry.update();


        }
    }
}