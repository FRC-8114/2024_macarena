package frc.robot.subsystems;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class SwerveOrchestraRequest implements SwerveRequest {
    private String m_music = "macarena.chrp";
    private Orchestra m_orchestra = new Orchestra();

    private boolean flagChanged = false;

    private boolean hasAddedInstruments = false;

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        StatusCode ret = StatusCode.OK;

        if (!hasAddedInstruments) {
            ret = addInstruments(modulesToApply);
            hasAddedInstruments = true;
        }

        if (flagChanged) {
            ret = m_orchestra.loadMusic(m_music);
            flagChanged = false;
        }

        return ret;
    }

    public StatusCode play() {
        return m_orchestra.play();
    }

    public StatusCode stop() {
        return m_orchestra.stop();
    }

    public StatusCode pause() {
        return m_orchestra.pause();
    }

    public SwerveOrchestraRequest withMusic() {

        flagChanged = true;
  
        return this;
    }

    private StatusCode addInstruments(SwerveModule... modules) {
        StatusCode retErr = StatusCode.OK;

        // Iterate over drive motors
        for(var module : modules) {
            var err = m_orchestra.addInstrument(module.getDriveMotor());
            if (err.isError()) {
                retErr = err;
            }
        }

        // Iterate over steer motors last
        for (var module : modules) {
            var err = m_orchestra.addInstrument(module.getSteerMotor());
            if (err.isError()) {
                retErr = err;
            }
        }

        return retErr;
    }
}
