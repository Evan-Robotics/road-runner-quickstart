package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;

@Config
@Autonomous(name="--Main-- Auto V1 (Blue Carousel)", group="drive")
public class Auto_V1_Blue_Carousel extends LinearOpMode {
    public static double dback1 = 15;
    public static double dleft1 = 23;
    public static double dforw1 = 5;
    public static double tforw1 = 0.8;

    public static double dback2 = 20;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot   = new HardwareRagnarok();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(-42, 63, Math.PI/2);

        drive.setPoseEstimate(startPose);

        Trajectory back1 = drive.trajectoryBuilder(startPose)
                .back(dback1)
                .build();

        Trajectory left1 = drive.trajectoryBuilder(back1.end())
                .strafeLeft(dleft1)
                .build();

        Trajectory forwards1 = drive.trajectoryBuilder(left1.end())
                .forward(dforw1)
                .build();

        Trajectory back2 = drive.trajectoryBuilder(forwards1.end())
                .back(dback2)
                .build();


        waitForStart();


        sleep(500);

        drive.followTrajectory(back1);
        drive.followTrajectory(left1);
        //drive.followTrajectory(forwards1);

        drive.setMotorPowers(0.2,0.2,0.2,0.2);
        sleep(Math.round(tforw1*1000));
        drive.setMotorPowers(0,0,0,0);

        robot.spinny_thing.setPower(-0.25);
        sleep(6000);
        robot.spinny_thing.setPower(0);

        drive.followTrajectory(back2);



        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
