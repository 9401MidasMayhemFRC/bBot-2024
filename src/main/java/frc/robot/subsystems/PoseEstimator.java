package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/*** Countine After GRC ***/ 

public class PoseEstimator extends SubsystemBase{
    private static final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator m_poseEstimator;
    private PhotonCamera m_cam;

    private boolean m_auto = true;

    private SwerveDrivePoseEstimator m_drivePose;
    private boolean m_validTag;
    private PhotonPipelineResult m_result;

    private Field2d m_field= new Field2d();

    private Drivetrain m_drive;

    public PoseEstimator(Drivetrain drive, PhotonCamera cam){
        m_drivePose = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                drive.getGyro(),
                drive.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.229, 0.229, 0.229),
                VecBuilder.fill(10, 10, 10));

        m_cam = cam;

        m_poseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_cam, VisionConstants.robotToCam);

        m_drive = drive;

        
        
    }

    public void teleopInit(){
        m_auto = false;
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_poseEstimator.update();
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose.getRotation());
        m_drivePose.resetPosition(m_drive.getGyro().times(1.0), m_drive.getModulePositions(), pose);
    }

    public Pose2d getPose() {

        return m_drivePose.getEstimatedPosition();

    }

    public Rotation2d getRotation2d(){
        return m_cam.getLatestResult().getMultiTagResult().estimatedPose.best.getRotation().toRotation2d();
    }

    @Override
    public void periodic() {

        m_result = m_cam.getLatestResult();
        m_validTag = m_result.hasTargets();

        int tagsDetected = m_result.getTargets().size();

        double area = 0.0;
        for(int i=0; i<tagsDetected; i++){
            area += m_result.getTargets().get(i).getArea();
        }

        double averageArea = area/tagsDetected;
        
        SmartDashboard.putNumber("Total Area of Tags", area);
        SmartDashboard.putNumber("Average Area of Tags", averageArea);

        double time = Timer.getFPGATimestamp();

        m_drivePose.updateWithTime(time, m_drive.getGyro(), m_drive.getModulePositions());

        double latency = m_result.getLatencyMillis()/1000;
        Pose2d m_pose;

        if (m_validTag && m_cam.isConnected() && (averageArea>0.1)){
            if (m_cam.getLatestResult().targets.size() > 1){
                
                Transform3d bestPose = m_cam.getLatestResult().getMultiTagResult().estimatedPose.best;


                var best =
                    new Pose3d()
                            .plus(bestPose) // field-to-camera
                            .relativeTo(m_aprilTagFieldLayout.getOrigin())
                            .plus(VisionConstants.robotToCam.inverse());


                m_pose = new Pose2d(best.getTranslation().toTranslation2d(), best.getRotation().toRotation2d());
                SmartDashboard.putString("Pose", m_pose.toString());
                SmartDashboard.putString("Camera Results", m_cam.getLatestResult().getMultiTagResult().toString());
                double antitrust = (1.6/averageArea);
                if (antitrust < 2.0){
                    antitrust = 2.0;
                }
                if(m_auto){
                    m_drivePose.addVisionMeasurement( m_pose , time-latency,VecBuilder.fill((antitrust)*10, (antitrust)*10, (antitrust)*50));
                } else {
                    m_drivePose.addVisionMeasurement( m_pose , time-latency,VecBuilder.fill(antitrust, antitrust, antitrust*5));
                }
                

            }else{

                if (averageArea > 0.25){
                
                    PhotonTrackedTarget lowestAmbiguityTarget = m_result.getBestTarget();

                    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

                    Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId); 

                    EstimatedRobotPose m_estimatedPose = new EstimatedRobotPose(
                            targetPosition
                                    .get()
                                    .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                                    .transformBy(VisionConstants.robotToCam.inverse()),
                            m_result.getTimestampSeconds(),
                            m_result.getTargets(),
                            PoseStrategy.LOWEST_AMBIGUITY);

                    m_pose = m_estimatedPose.estimatedPose.toPose2d();
                    SmartDashboard.putString("Pose", m_pose.toString());
                    SmartDashboard.putString("Camera Results", m_cam.getLatestResult().toString());
                    double antitrust = (1.6/averageArea);
                    if (antitrust < 2.0){
                        antitrust = 2.0;
                    }
                    if(m_auto){
                        m_drivePose.addVisionMeasurement( m_pose , time-latency,VecBuilder.fill(antitrust*10, antitrust*10, antitrust*50));
                    } else {
                        m_drivePose.addVisionMeasurement( m_pose , time-latency,VecBuilder.fill(antitrust, antitrust, antitrust*5));
                    }
                }
            }
        } else {
            if (!m_cam.isConnected()){
                DriverStation.reportError("Camera Disconnected", true);
            } else {
                DriverStation.reportWarning("No Tags Read/Trusted", false);
            }
        }

        m_field.setRobotPose(m_drivePose.getEstimatedPosition());
        SmartDashboard.putData(m_field);

    }
}
