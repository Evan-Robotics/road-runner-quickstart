package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.HardwareRagnarok;

import java.util.Collections;

@TeleOp(name = "--MAIN-- TeleOp V1", group = "Ragnarok")
public class TeleOp_V1 extends LinearOpMode {

    // Driving variables
    enum Mode {
        NORMAL_CONTROL,
        TANK_CONTROL,
        GRID          // testing
    }

    boolean spinny_direction = false;
    boolean spinny_direction_toggler = false;

    double capPos = 0;

    double speedChange1;
    double speedChange2;
    double intake_speed;

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    //Target Position of Storage Unit
    private Vector2d storage_pos = new Vector2d(-66, 37);

    //Timers
    private final ElapsedTime motorTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        waitForStart();

        ////VARIABLES\\\\\
        // (for servos, when we have them)

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            //Gamepad Input
            double ly;
            double lx;
            double rx;

            //Initialize Localizer
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            //Initialize FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            //TelemetryPacket packet = new TelemetryPacket();

            //// Gamepad Controls \\\\

            // Speed change
            if (gamepad1.left_bumper) {
                speedChange1 = 0.5;
            } else if (gamepad1.right_bumper) {
                speedChange1 = 1;
            } else {
                speedChange1 = 0.7;
            }

            if (gamepad2.left_bumper) {
                speedChange2 = 0.5;
            } else if (gamepad1.right_bumper) {
                speedChange2 = 1;
            } else {
                speedChange2 = 0.7;
            }

            // y toggles direction of spinny thing
            if (spinny_direction_toggler && !gamepad2.y) {
                spinny_direction = !spinny_direction;
            }
            spinny_direction_toggler = gamepad2.y;

            robot.spinny_thing.setPower(spinny_direction ? speedChange2 * gamepad2.left_trigger : -1 * speedChange2 * gamepad2.left_trigger);

            // right trigger controls intake
            intake_speed = gamepad2.right_trigger * speedChange2 * ( gamepad2.left_bumper ? -1 : 1 );

            robot.intake_main.setPower(intake_speed * 0.75);
            robot.intake2.setPower(intake_speed);
            robot.intake3.setPower(intake_speed);

            robot.bucket.setPosition( gamepad2.a ? 0.4 : 0 );

            robot.hook.setPosition( gamepad2.x ? 0.6 : 0 );

            if (gamepad1.start && gamepad1.dpad_up) {
                drive.getLocalizer().setPoseEstimate(new Pose2d(-63, 60, Math.PI/2));
            }

            if (-gamepad2.left_stick_y > 0) {
                robot.lift.setPower(-gamepad2.left_stick_y * speedChange2 * 0.75);
            }
            else {
                robot.lift.setPower(-gamepad2.left_stick_y * speedChange2 * 0.25);
            }

            capPos += gamepad2.right_stick_y * 0.01 * (speedChange2 * speedChange2);
            capPos = Math.max(Math.min(0.4, capPos), 0);

            robot.cap.setPosition( capPos );

            //Distance to Tower


            switch (currentMode) {
                case NORMAL_CONTROL:

                    ly = -gamepad1.left_stick_y;
                    lx = -gamepad1.left_stick_x;
                    rx = -gamepad1.right_stick_x;

                    //Normal Robot Control
                    driveDirection = new Pose2d(
                            Math.abs(ly) * ly * speedChange1,
                            Math.abs(lx) * lx * speedChange1,
                            Math.abs(rx) * rx * speedChange1
                    );

                    // Switch to tank if gamepad1 left dpad is activated
                    if (gamepad1.dpad_left) {
                        currentMode = Mode.GRID;
                    }
                    break;

                case GRID: // Switch to normal control if gamepad1 dpad right is activated

                    if (gamepad1.dpad_right) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    ly = -gamepad1.left_stick_y;
                    lx = -gamepad1.left_stick_x;
                    rx = -gamepad1.right_stick_x;

                    Vector2d fieldFrameInput = new Vector2d(
                            Math.signum(ly) * Math.pow(ly, 2) * speedChange1,
                            Math.signum(lx) * Math.pow(lx, 2) * speedChange1
                    );

                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                    //headingController.setTargetPosition(rx * speedChange1);

                    double headingInput = Math.signum(rx) * Math.pow(rx, 2) * speedChange1;

                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    if (inObstacle(poseEstimate.plus(driveDirection))) {
                        telemetry.addData("Collision", "Detected");
                    } else {
                        telemetry.addData("Collision", "Not Detected");
                    }

                    break;

            }

            //Dashboard View
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            //Driver Station Telemetry

            telemetry.addData("mode", currentMode);
            telemetry.addData("Spinner Direction", spinny_direction ? "Red" : "Blue");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Max Power", Collections.max(drive.getWheelVelocities()));
            telemetry.addData("Lift Power", robot.lift.getPower());
            telemetry.update();
        }
    }

    private boolean inObstacle(Pose2d pos) {
        if ((pos.getX() > 18 && pos.getX() < 30) && (pos.getY() > -54 && pos.getY() < 54)) {
            return true;
        }
        else if ((pos.getX() > 24 && pos.getX() < 54) && (pos.getY() > -30 && pos.getY() < -18)) {
            return true;
        }
        else if (false) {
            return true;
        }

        return false;
    }
}
