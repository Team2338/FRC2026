// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.robot.commands.Agitator.AgitatorAuto;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoCollectRPM;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPercent;
import team.gif.robot.commands.Collector.CollectorAutos.CollectorAutoPivotDown;
import team.gif.robot.commands.SequentialAutoCommands.CollectAutoSequence;
import team.gif.robot.commands.SequentialAutoCommands.ShootAutoSequence;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoBack;
import team.gif.robot.commands.Shooter.ShooterAutos.IndexerAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.PreIndexerAutoPercent;
import team.gif.robot.commands.Shooter.ShooterAutos.ShooterAutoRPM;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("Agitator Auto", new CollectorAutoPercent(0.5).alongWith(new AgitatorAuto()));
        NamedCommands.registerCommand("Collector Down", new CollectorAutoPivotDown());
        NamedCommands.registerCommand("Collector Collect", new CollectorAutoCollectRPM());
        NamedCommands.registerCommand("Indexer Run", new IndexerAutoBack(0.25).andThen(new PreIndexerAutoPercent().alongWith(new IndexerAutoPercent())));
        NamedCommands.registerCommand("Shoot", new ShooterAutoRPM());
        NamedCommands.registerCommand("Collect Sequence", new CollectAutoSequence());
        NamedCommands.registerCommand("Shoot Sequence", new ShootAutoSequence());
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
    }
}
