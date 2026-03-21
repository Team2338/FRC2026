package team.gif.robot.subsystems.drivers.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import team.gif.lib.LimelightHelpers;
import team.gif.lib.drivePace;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotorFactory;
import team.gif.robot.subsystems.drivers.swerve.utilities.Encoder;
import team.gif.robot.subsystems.drivers.swerve.utilities.EncoderFactory;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveMap;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotorFactory;

import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;


@SuppressWarnings("unused")
public class SwerveDrivetrain extends SubsystemBase {
    //region Setup
    //--------------------
    //-------Utility------
    //--------------------
    private final SwerveMap deviceMap;
    private final SwerveConstants constants;
    private final DriveMotorFactory driveMotorFactory;
    private final TurnMotorFactory turnMotorFactory;
    private final EncoderFactory encoderFactory;
    private drivePace drivePace;
    private boolean isRedAlliance = false;
    public boolean debugMode = false;
    private Translation3d[] moduleLocations;

    //--------------------
    //-------Devices------
    //--------------------
    public SwerveModule fL;
    public SwerveModule fR;
    public SwerveModule rL;
    public SwerveModule rR;

    private DriveMotor fLDriveMotor;
    private DriveMotor fRDriveMotor;
    private DriveMotor rLDriveMotor;
    private DriveMotor rRDriveMotor;

    private TurnMotor fLTurnMotor;
    private TurnMotor fRTurnMotor;
    private TurnMotor rLTurnMotor;
    private TurnMotor rRTurnMotor;

    private Encoder fLEncoder;
    private Encoder fREncoder;
    private Encoder rLEncoder;
    private Encoder rREncoder;


    //--------------------
    //-------Odometry-----
    //--------------------
    public SwerveDrivePoseEstimator poseEstimator;
    private boolean skidding = false;
    private boolean collision = false;
    private boolean odometryReady = false;
    private static final Rotation2d oneEighty = Rotation2d.fromDegrees(180);

    //--------------------
    //-------Vision-------
    //--------------------
    private boolean visionEnabled = true;
    private String[] limelightNames = new String[] {};
    private PhotonCamera[] photonCameras = new PhotonCamera[] {};
    private PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[] {};
    private static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private Matrix<N3, N1> photonCurrStdDevs;


    //--------------------
    //-------Logging------
    //--------------------
    private ChassisSpeeds targetState = new ChassisSpeeds();

    // Network Table publishers for the swerve
    // states so that we can use them in advantage scope
    private static final StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetSwerveState", SwerveModuleState.struct).publish();
    private static final StructArrayPublisher<SwerveModuleState> actualPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ActualSwerveState", SwerveModuleState.struct).publish();
    private static final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedPose", Pose2d.struct).publish();
    private static final StructPublisher<Pose2d> estPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedVisionPose", Pose2d.struct).publish();
    private static final StructPublisher<ChassisSpeeds> chassisSpeedsStructPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();

    //endregion

    /**
     * Constructs a SwerveDrivetrain with the specified configuration
     * @param config The SwerveConfig for the drivetrain
     */
    public SwerveDrivetrain(SwerveConfiguration config) {
        super();

        this.deviceMap = config.deviceMap;
        this.constants = config.constants;
        this.driveMotorFactory = config.driveMotorFactory;
        this.turnMotorFactory = config.turnMotorFactory;
        this.encoderFactory = config.encoderFactory;

        configModules();

        resetDriveEncoders();

        Rotation2d rotation = Robot.pigeon.getRotation2d();

        if(checkRedAlliance()) {
            rotation = rotation.rotateBy(oneEighty);
        }

        poseEstimator = new SwerveDrivePoseEstimator(constants.DRIVE_KINEMATICS, rotation, getSwerveModulePositions(), new Pose2d(0, 0, rotation));

        drivePace = team.gif.lib.drivePace.COAST_FR;

        configPathPlanner();
    }

    /**
     * Periodic function
     * - constantly update the odometry
     */
    @Override
    public void periodic() {
        //Have to do skid detection and collision first
        //so we don't update with bad data
        checkSkidding();
        checkCollision();

        isRedAlliance = checkRedAlliance();
        Rotation2d rotation = Robot.pigeon.getRotation2d();
        if(isRedAlliance) {
            rotation = rotation.rotateBy(oneEighty);
        }

        if(!skidding && !collision) {
            poseEstimator.update(
                    rotation,
                    getSwerveModulePositions()
            );
        } else {
            //This is set false as long as the robot is skidding or
            //has high accel. This is overridden if a tag is visible.
            odometryReady = false;
        }

        if (Robot.pigeon.getYawRate() < 720 && visionEnabled) {
            addLimelightEstimates();
            addPhotonEstimates();
        }

        if (debugMode) {
            posePublisher.set(poseEstimator.getEstimatedPosition());
            updateShuffleboardDebug();
        }
    }

    //region Driving
    /**
     * Drive the bot with given params - always field relative
     *
     * @param x   dForward
     * @param y   dLeft
     * @param rot dRot
     */
    public void drive(double x, double y, double rot) {
        targetState = drivePace.getIsFieldRelative() ?
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                : new ChassisSpeeds(x, y, rot);

        SwerveModuleState[] swerveModuleStates = constants.DRIVE_KINEMATICS.toSwerveModuleStates(targetState);

        if (debugMode) {
            SwerveModuleState[] actualStates = {fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            targetPublisher.set(swerveModuleStates);
            actualPublisher.set(actualStates);
        }
        setModuleStates(swerveModuleStates);
    }

    /**
    * Get the robot relative speed
    * @return ChassisSpeeds of the robot relative speed
    */
    public ChassisSpeeds getRobotRelativeSpeed() {
        SwerveModuleState[] states = getSwerveModuleStates();

        ChassisSpeeds speed = constants.DRIVE_KINEMATICS.toChassisSpeeds(states[0], states[1], states[2], states[3]);

        if (debugMode) {
            chassisSpeedsStructPublisher.set(speed);
        }

        return speed;
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Robot Relative ChassisSpeeds to apply to wheel speeds
     * @implNote Use only in {@link SwerveDrivetrain}
     */
    public void setModuleChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        targetState = chassisSpeeds;

        SwerveModuleState[] swerveModuleStates = constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, drivePace.getValue()
        );

        for (SwerveModuleState state : swerveModuleStates) {
            state.speedMetersPerSecond = Math.min(state.speedMetersPerSecond, drivePace.getValue());
        }

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);

        if(debugMode) {
            SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            actualPublisher.set(actualStates);
            chassisSpeedsStructPublisher.set(chassisSpeeds);
            targetPublisher.set(swerveModuleStates);
        }
    }

    /**
     * Get the robot relative speed
     * @return ChassisSpeeds of the robot relative speed
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState frontLeftState = new SwerveModuleState(fL.getDriveVelocity(), Rotation2d.fromDegrees(fL.getTurningHeadingDegrees()));
        SwerveModuleState frontRightState = new SwerveModuleState(fR.getDriveVelocity(), Rotation2d.fromDegrees(fR.getTurningHeadingDegrees()));
        SwerveModuleState rearLeft = new SwerveModuleState(rL.getDriveVelocity(), Rotation2d.fromDegrees(rL.getTurningHeadingDegrees()));
        SwerveModuleState rearRight = new SwerveModuleState(rR.getDriveVelocity(), Rotation2d.fromDegrees(rR.getTurningHeadingDegrees()));

        return new SwerveModuleState[]{frontLeftState, frontRightState, rearLeft, rearRight};
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     *
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     * @implNote Only for use in the SwerveDrivetrain class and by pathplanner, for any general use {@link SwerveDrivetrain#drive(double x, double y, double rot)}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Get the current position of each of the swerve modules
     *
     * @return An array in form fL -> fR -> rL -> rR of each of the module positions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {

        return new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    /**
     * This set moves all the modules to 90 degrees. It turns the modules inward to prevent the robot from moving
     */
    public void modulesTo90() {
        SwerveModuleState state90 = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
        fL.setDesiredState(state90, true);
        fR.setDesiredState(state90, false);
        rL.setDesiredState(state90, true);
        rR.setDesiredState(state90, false);

    }

    public void setMaxDrive() {
        fLDriveMotor.set(1);
        fRDriveMotor.set(1);
        rLDriveMotor.set(1);
        rRDriveMotor.set(1);
    }

    public void stopDrive() {
        drive(0,0,0);
    }

    /**
     * Stop all the modules
     */
    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    /**
     * Reset the drive encoders
     */
    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * Get the current drivePace settings
     *
     * @return the current drivePace settings
     */
    public drivePace getDrivePace() {
        return drivePace;
    }

    /**
     * set the drivePace settings for the drivetrain
     *
     * @param drivePace the drivePace to set
     */
    public void setDrivePace(drivePace drivePace) {
        this.drivePace = drivePace;
    }
    //endregion

    //region Odometry
    /**
     * Reset the odometry to a given pose
     *
     * @param pose the pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(Robot.pigeon.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    /**
     * Compares the translational component of the wheel velocities to each other to detect skidding
     */
    private void checkSkidding() {
        //Skid detection
        //The skid detection works by first separating how much of each
        //wheel speed contributes to translation vs rotation
        //Then they are compared to find the difference between the maximum and the minimum
        //If the difference is over the threshold the odometry is not updated with encoders
        //and the odometry is invalidated
        SwerveModuleState[] moduleStates = getSwerveModuleStates();
        double[] translationVelocities = getSwerveModuleTranslationSpeed(targetState, moduleStates);
        double min = 100 , max = 0;
        for (double v : translationVelocities) {
            min = Math.min(v, min);
            max = Math.max(v, max);
        }

        skidding = (max-min) > constants.SKID_THRESHOLD;
        if (debugMode) {
            SmartDashboard.putBoolean("Skid", skidding);
            SmartDashboard.putNumber("Delta", (max - min));
            SmartDashboard.putNumberArray("Wheel Translations", translationVelocities);
        }
    }

    /**
     * Uses the pigeon to check the acceleration of the bot to detect collisions
     */
    private void checkCollision() {
        //Collision Detection
        //this will also detect hard stops and starts,
        //as it is likely that these may also cause issues.
        double accel = Robot.pigeon.getAcceleration();
        if(debugMode) SmartDashboard.putNumber("Acceleration", accel);
        //accel is measured in g's.
        collision = accel > constants.COLLISION_THRESHOLD;
    }

    /**
     * Uses the SKID_THRESHOLD constant to determine if the robot is skidding
     * @return Returns true if the robot is currently skidding
     */
    public boolean isSkidding() {
        return skidding;
    }

    /**
     * Uses the COLLISION_THRESHOLD constant to determine if the robot is in a collision
     * @return Returns true if the robot is currently in a collision
     */
    public boolean isCollision() {
        return collision;
    }

    /**
     * Checks if there has been a vision updates since a skid
     * or collision was detected
     * @return Returns true if the odometry is probably accurate
     */
    public boolean isOdometryReady() {
        return odometryReady;
    }

    /**
     * Get the current pose of the robot
     *
     * @return The current pose of the robot (Pose2D)
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * This function separates the translation speed from the overall speed.
     * The math is based on two white-papers: <a href="https://www.chiefdelphi.com/uploads/short-url/qzj4k2LyBs7rLxAem0YajNIlStH.pdf">one</a> and <a href="https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf">two</a>
     * @param chassisSpeeds  the actual chassis speeds of the robot
     * @param s  the current swerve module states
     * @return  an array of doubles with the transitive speeds in meters per second. The order is the same as provided in swerve module states
     */
    private double[] getSwerveModuleTranslationSpeed(ChassisSpeeds chassisSpeeds, SwerveModuleState[] s) {
        //This isn't really the intended use of the
        //Translation3d, I just need something that represents a vector
        //the vector dimensions are <x, y, rotation>
        //This is the angular velocity <0, 0, omega>
        Translation3d omega = new Translation3d(0, 0, chassisSpeeds.omegaRadiansPerSecond);


        return IntStream.range(0, s.length)
                .mapToDouble(i -> {
                    SwerveModuleState state = s[i];

                    Translation3d moduleVelocity =
                            new Translation3d(state.speedMetersPerSecond,
                                    new Rotation3d(state.angle));

                    Translation3d moduleLocation = moduleLocations[i];
                    Translation3d cross = new Translation3d(omega.cross(moduleLocation));
                    Translation3d t = moduleVelocity.minus(cross);

                    return t.getNorm();
                }).toArray();
    }
    //endregion

    //region Vision
    /**
     * Set the limelight enabled status
     * @param enabled - enable limelight vision updates
     */
    public void setVisionEnabled(boolean enabled) {
        visionEnabled = enabled;
    }

    /**
     * Get the limelight enabled status
     * @return boolean for the current limelight enabled status
     */
    public boolean getVisionEnabled() {
        return visionEnabled;
    }

    /**
     * Add a limelight to the list of limelights used for vision based odometry
     * @param newLimelightName - the name of the limelight to add
     */
    public void addLimelight(String newLimelightName) {
        String[] newArray = new String[limelightNames.length + 1];
        System.arraycopy(limelightNames, 0, newArray, 0, limelightNames.length);
        newArray[newArray.length - 1] = newLimelightName;
        limelightNames = newArray;
    }

    /**
     * Get the list of limelights used for vision based odometry
     * @return String array of limelight names
     */
    public String[] getLimelightNames() {
        return limelightNames;
    }

    private void addLimelightEstimates() {
        for (String limelightName : limelightNames) {
            LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
            if (visionEnabled && estimate != null && estimate.tagCount > 0) {
                poseEstimator.addVisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds,
                        VecBuilder.fill(.7, .7, 9999999));
                odometryReady = true;
            }
        }
    }

    /**
     * Adds a photon vision camera to be used for vision based odometry
     * @param newPhotonCamName the name of the camera defined in the PhotonVision UI
     * @param cameraLocation the location of the camera, relative to the center of the bot
     */
    public void addPhotonCamera(String newPhotonCamName, Transform3d cameraLocation) {
        PhotonCamera[] newArray = new PhotonCamera[photonCameras.length + 1];
        System.arraycopy(photonCameras, 0, newArray, 0, photonCameras.length);
        newArray[newArray.length - 1] = new PhotonCamera(newPhotonCamName);
        photonCameras = newArray;

        PhotonPoseEstimator[] newEstArray = new PhotonPoseEstimator[photonPoseEstimators.length + 1];
        System.arraycopy(photonPoseEstimators, 0, newEstArray, 0, photonPoseEstimators.length);
        newEstArray[newEstArray.length - 1] = new PhotonPoseEstimator(tagLayout, cameraLocation);
        photonPoseEstimators = newEstArray;
    }

    /**
     * Get the list of PhotonVision cameras currently registered
     * @return An array of PhotonCameras
     */
    public PhotonCamera[] getPhotonCameras() {
        return photonCameras;
    }

    /**
     * Get the list of PhotonVision pose estimators currently registered
     * @return An array of PhotonPoseEstimators
     */
    public PhotonPoseEstimator[] getPhotonPoseEstimators() {
        return photonPoseEstimators;
    }

    private void addPhotonEstimates() {
        for (var i = 0; i < photonCameras.length; i++) {
            PhotonCamera camera = photonCameras[i];
            PhotonPoseEstimator estimator = photonPoseEstimators[i];
            Optional<EstimatedRobotPose> visionEst;
            for (var result : camera.getAllUnreadResults()) {
                visionEst = estimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }

                updateEstimationStdDevs(visionEst, result.getTargets(), estimator);

                visionEst.ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();

                            if (debugMode) {
                                estPublisher.set(est.estimatedPose.toPose2d());
                            }

                            Rotation2d estRotation = isRedAlliance ? est.estimatedPose.getRotation().toRotation2d().rotateBy(oneEighty) : est.estimatedPose.getRotation().toRotation2d();
                            var newPose = new Pose2d(est.estimatedPose.getX(), est.estimatedPose.getY(), estRotation);
                            poseEstimator.addVisionMeasurement(newPose, est.timestampSeconds, estStdDevs);
                            odometryReady = true;
                        });
            }

        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            photonCurrStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                photonCurrStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 999999);
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                photonCurrStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from
     * Photonvision, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return photonCurrStdDevs;
    }

    //endregion

    //region Configuration

    /**
     * Gets the constants for this SwerveDrivetrain
     * @return the {@link SwerveConstants} passed into the {@link SwerveConfiguration}
     */
    public SwerveConstants getConstants() {
        return constants;
    }

    private boolean checkRedAlliance() {
        //This state should never happen unless we are not connected.
        //It is set to red because that is what we are set up for in the shop.
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return true;
        }

        return DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() != DriverStation.Alliance.Blue;
    }

    private void configModules() {

        fLDriveMotor = driveMotorFactory.create(deviceMap.FRONT_LEFT_DRIVE_MOTOR_ID, constants);
        fRDriveMotor = driveMotorFactory.create(deviceMap.FRONT_RIGHT_DRIVE_MOTOR_ID, constants);
        rLDriveMotor = driveMotorFactory.create(deviceMap.REAR_LEFT_DRIVE_MOTOR_ID, constants);
        rRDriveMotor = driveMotorFactory.create(deviceMap.REAR_RIGHT_DRIVE_MOTOR_ID, constants);

        fLTurnMotor = turnMotorFactory.create(deviceMap.FRONT_LEFT_TURNING_MOTOR_ID);
        fRTurnMotor = turnMotorFactory.create(deviceMap.FRONT_RIGHT_TURNING_MOTOR_ID);
        rLTurnMotor = turnMotorFactory.create(deviceMap.REAR_LEFT_TURNING_MOTOR_ID);
        rRTurnMotor = turnMotorFactory.create(deviceMap.REAR_RIGHT_TURNING_MOTOR_ID);

        // In case the turn motor is a TalonSRX that has the encoder connected to the data port
        if(encoderFactory == null && fLTurnMotor instanceof TalonSRXTurnMotorEncoder) {
            fLEncoder = (Encoder) fLTurnMotor;
            fREncoder = (Encoder) fRTurnMotor;
            rLEncoder = (Encoder) rLTurnMotor;
            rREncoder = (Encoder) rRTurnMotor;
        } else {
            assert encoderFactory != null;
            fLEncoder = encoderFactory.create(deviceMap.FRONT_LEFT_ENCODER_ID);
            fREncoder = encoderFactory.create(deviceMap.FRONT_RIGHT_ENCODER_ID);
            rLEncoder = encoderFactory.create(deviceMap.REAR_LEFT_ENCODER_ID);
            rREncoder = encoderFactory.create(deviceMap.REAR_RIGHT_ENCODER_ID);
        }


        fL = new SwerveModule(
                fLDriveMotor,
                fLTurnMotor,
                fLEncoder,
                constants.FL_TURN_INVERTED,
                constants.FL_DRIVE_INVERTED,
                constants.FRONT_LEFT_OFFSET,
                constants.FL_DRIVE_FF,
                constants.FL_FF,
                constants.FL_P,
                constants.MAX_DRIVE_TEMP
        );

        fR = new SwerveModule (
                fRDriveMotor,
                fRTurnMotor,
                fREncoder,
                constants.FR_TURN_INVERTED,
                constants.FR_DRIVE_INVERTED,
                constants.FRONT_RIGHT_OFFSET,
                constants.FR_DRIVE_FF,
                constants.FR_FF,
                constants.FR_P,
                constants.MAX_DRIVE_TEMP
        );

        rL = new SwerveModule (
                rLDriveMotor,
                rLTurnMotor,
                rLEncoder,
                constants.RL_TURN_INVERTED,
                constants.RL_DRIVE_INVERTED,
                constants.REAR_LEFT_OFFSET,
                constants.RL_DRIVE_FF,
                constants.RL_FF,
                constants.RL_P,
                constants.MAX_DRIVE_TEMP
        );

        rR = new SwerveModule (
                rRDriveMotor,
                rRTurnMotor,
                rREncoder,
                constants.RR_TURN_INVERTED,
                constants.RR_DRIVE_INVERTED,
                constants.REAR_RIGHT_OFFSET,
                constants.RR_DRIVE_FF,
                constants.RR_FF,
                constants.RR_P,
                constants.MAX_DRIVE_TEMP
        );

        Translation2d[] moduleLocations2d = constants.DRIVE_KINEMATICS.getModules();
        moduleLocations = new Translation3d[]{new Translation3d(moduleLocations2d[0]),
                new Translation3d(moduleLocations2d[1]),
                new Translation3d(moduleLocations2d[2]),
                new Translation3d(moduleLocations2d[3])};
    }

    private void configPathPlanner() {
        RobotConfig ppConfig;
        try{
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ModuleConfig moduleConfig = constants.PATHPLANNER_MODULE_CONFIG;
            ppConfig = new RobotConfig(constants.MASS_KG, constants.MOI_KGM2, moduleConfig, constants.TRACK_WIDTH_METERS);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::setModuleChassisSpeeds,
                constants.AUTO_DRIVE_CONTROLLER,
                ppConfig,
                this::checkRedAlliance,
                this
        );
    }

    //endregion

    //region Reporting (Temps and Debug)
    public double fLDriveTemp() { return fLDriveMotor.getTemp(); }
    public double fRDriveTemp() { return fRDriveMotor.getTemp(); }
    public double rLDriveTemp() { return rLDriveMotor.getTemp(); }
    public double rRDriveTemp() { return rRDriveMotor.getTemp(); }

    /**
     * Enables debug mode. Adds the following to the network table:
     * <ul>
     *     <li>Heading for each module (Gyro Widget)</li>
     *     <li>Actual Swerve State (Elastic Widget)</li>
     *     <li>Target Swerve State (Elastic Widget)</li>
     *     <li>Actual Swerve State (AdvantageScope)</li>
     *     <li>Target Swerve State (AdvantageScope)</li>
     *     <li>Target Chassis Speed (AdvantageScope</li>
     *     <li>Raw Encoder Readings</li>
     *     <li>Raw Encoder Degrees</li>
     *     <li>Raw Encoder Radians</li>
     *     <li>Raw Drive Encoder</li>
     * </ul>
     */
    public void enableDebugMode() {
        debugMode = true;
        SmartDashboard.putData("Swerve/FL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/FR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fR::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/RL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/RR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rR::getTurningHeadingDegrees, null);
        });

        SmartDashboard.putData("Swerve/Actual Swerve State", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> fL.getTurningHeading(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> fL.getDriveVelocity(), null);

            builder.addDoubleProperty("Front Right Angle", () -> fR.getTurningHeading(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> fR.getDriveVelocity(), null);

            builder.addDoubleProperty("Back Left Angle", () -> rL.getTurningHeading(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> rL.getDriveVelocity(), null);

            builder.addDoubleProperty("Back Right Angle", () -> rR.getTurningHeading(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> rR.getDriveVelocity(), null);

            builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(Robot.pigeon.get360Heading()), null);
        });

        SmartDashboard.putData("Swerve/Target Swerve State", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", fL::getTargetAngle, null);
            builder.addDoubleProperty("Front Left Velocity", fL::getTargetVelocity, null);

            builder.addDoubleProperty("Front Right Angle", fR::getTargetAngle, null);
            builder.addDoubleProperty("Front Right Velocity", fR::getTargetVelocity, null);

            builder.addDoubleProperty("Back Left Angle", rL::getTargetAngle, null);
            builder.addDoubleProperty("Back Left Velocity", rL::getTargetVelocity, null);

            builder.addDoubleProperty("Back Right Angle", rR::getTargetAngle, null);
            builder.addDoubleProperty("Back Right Velocity", rR::getTargetVelocity, null);

            builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(Robot.pigeon.get360Heading()), null);
        });
    }
    private void updateShuffleboardDebug() {
        SmartDashboard.putNumber("Swerve/FR Raw Degrees", fR.encoderDegrees());
        SmartDashboard.putNumber("Swerve/FL Raw Degrees", fL.encoderDegrees());
        SmartDashboard.putNumber("Swerve/RR Raw Degrees", rR.encoderDegrees());
        SmartDashboard.putNumber("Swerve/RL Raw Degrees", rL.encoderDegrees());

        SmartDashboard.putNumber("Swerve/FL Raw Encoder", fLEncoder.getTicks());
        SmartDashboard.putNumber("Swerve/FR Raw Encoder", fREncoder.getTicks());
        SmartDashboard.putNumber("Swerve/RL Raw Encoder", rLEncoder.getTicks());
        SmartDashboard.putNumber("Swerve/RR Raw Encoder", rREncoder.getTicks());

        SmartDashboard.putNumber("Swerve/FR Raw Radians", fR.getTurningHeading());
        SmartDashboard.putNumber("Swerve/FL Raw Radians", fL.getTurningHeading());
        SmartDashboard.putNumber("Swerve/RR Raw Radians", rR.getTurningHeading());
        SmartDashboard.putNumber("Swerve/RL Raw Radians", rL.getTurningHeading());

        SmartDashboard.putNumber("Swerve/FL Drive Encoder", fLDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/FR Drive Encoder", fRDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/RL Drive Encoder", rLDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/RR Drive Encoder", rRDriveMotor.getPosition());

        SmartDashboard.putNumber("Swerve/FL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/FR Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/RL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/RR Rot Output", fLTurnMotor.getOutput());
    }
    //endregion

    //region System Identification
    public SysIdRoutine getSysIdRoutine(String motors) {
        MutVoltage voltMut = Volts.mutable(0);


        if (motors.equals("drive")) {
            MutDistance posMut = Meters.mutable(0);
            MutLinearVelocity vMut= MetersPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(null, voltMut.mut_replace(8, Volts), null),
                    new SysIdRoutine.Mechanism(
                            voltage -> {
                                fLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                fL.turnHoldZero();
                                fRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                fR.turnHoldZero();
                                rLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rL.turnHoldZero();
                                rRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rR.turnHoldZero();
                            },
                            log -> {
                                log.motor("fLDrive")
                                        .voltage(voltMut.mut_replace(fLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("fRDrive")
                                        .voltage(voltMut.mut_replace(fRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fRDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rLDrive")
                                        .voltage(voltMut.mut_replace(rLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rRDrive")
                                        .voltage(voltMut.mut_replace(rRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rRDriveMotor.getVelocity(), MetersPerSecond));

                            },
                            this));
        } else if (motors.equals("turn")) {
            MutAngle thetaMut = Radians.mutable(0);
            MutAngularVelocity thetaVMut = RadiansPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                    voltage -> {
                            fLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            fRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                        }, log -> {
                            log.motor("fLTurn")
                                    .voltage(voltMut.mut_replace(fLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("fRTurn")
                                    .voltage(voltMut.mut_replace(fRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fREncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rLTurn")
                                    .voltage(voltMut.mut_replace(rLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rRTurn")
                                    .voltage(voltMut.mut_replace(rRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rREncoder.getVelocity(), RadiansPerSecond));
                    }, this));

        } else {
            DriverStation.reportError("Invalid motor type at SwerveDrivetrainMk4.getSysIdRoutine", false);
            return null;
        }
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param motor The motor to run the test on either "drive" or "turn"
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param motor The motor to run the test on either "drive" or "turn"
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).dynamic(direction);
    }
    //endregion
}