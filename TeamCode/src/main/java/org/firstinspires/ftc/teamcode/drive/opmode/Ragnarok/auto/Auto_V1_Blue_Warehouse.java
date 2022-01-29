package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="--Main-- Auto V1 (Blue Warehouse)", group="drive")
public class Auto_V1_Blue_Warehouse extends LinearOpMode {
    public static double dforw1 = 40;
    public static double drigh1 = 20;
    public static double dforw2 = 28;
    public static double dturn1 = 200;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot   = new HardwareRagnarok();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0, 63, 0);

        drive.setPoseEstimate(startPose);

        Trajectory forwards1 = drive.trajectoryBuilder(startPose)
                .forward(dforw1)
                .build();

        Trajectory righ1 = drive.trajectoryBuilder(forwards1.end())
                .strafeRight(drigh1)
                .build();

        Trajectory forwards2 = drive.trajectoryBuilder(righ1.end())
                .forward(dforw2)
                .build();

        TrajectorySequence turn1 = drive.trajectorySequenceBuilder(forwards2.end())
                .turn(Math.toRadians(dturn1))
                .build();



        waitForStart();


        sleep(500);

        drive.followTrajectory(forwards1);
        drive.followTrajectory(righ1);
        drive.followTrajectory(forwards2);
        drive.followTrajectorySequence(turn1);


        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
