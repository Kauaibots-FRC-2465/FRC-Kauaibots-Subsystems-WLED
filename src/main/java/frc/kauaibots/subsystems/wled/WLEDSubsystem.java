package frc.kauaibots.subsystems.wled;

import java.net.DatagramSocket;
import java.net.SocketException;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WLEDSubsystem extends SubsystemBase {
    private static final double NOTIFIER_INTERVAL_SECONDS = 0.02;

    private final DatagramSocket socket;
    private final Notifier playbackNotifier;
    private final AtomicBoolean playbackNotifierStarted;
    private final Animator marqueeAnimator;
    private final Animator horizontalStripAnimator;
    private final Animator imageAnimator;
    private final Animator gifAnimator;

    private volatile PlaybackState activePlayback;

    private static final class PlaybackState {
        private final AnimationUDPs animation;
        private double frameStartTimeSeconds;
        private int currentFrameIndex;
        private int lastSentFrameIndex;

        private PlaybackState(AnimationUDPs animation) {
            this.animation = animation;
            this.frameStartTimeSeconds = Timer.getFPGATimestamp();
            this.currentFrameIndex = 0;
            this.lastSentFrameIndex = -1;
        }
    }

    public WLEDSubsystem(WLEDConfig config) {
        marqueeAnimator = new MarqueeAnimator(config);
        horizontalStripAnimator = new HorizontalStripAnimator(config);
        imageAnimator = new ImageAnimator(config);
        gifAnimator = new GifAnimator(config);
        socket = createSocket();
        playbackNotifier = createPlaybackNotifier();
        playbackNotifierStarted = new AtomicBoolean(false);
        activePlayback = null;
    }

    @Override
    public void periodic() {
    }

    public Command showBitmapAsMarquee(String path) {
        AnimationUDPs animationUDPs = marqueeAnimator.animate(path);
        return runOnce(() -> setActivePlayback(animationUDPs)).ignoringDisable(true);
    }

    public Command showFromBitmapAnimationStrip(String path) {
        AnimationUDPs animationUDPs = horizontalStripAnimator.animate(path);
        return runOnce(() -> setActivePlayback(animationUDPs)).ignoringDisable(true);
    }

    public Command showBitmap(String path) {
        AnimationUDPs animationUDPs = imageAnimator.animate(path);
        return runOnce(() -> setActivePlayback(animationUDPs)).ignoringDisable(true);
    }

    public Command showGif(String path) {
        AnimationUDPs animationUDPs = gifAnimator.animate(path);
        return runOnce(() -> setActivePlayback(animationUDPs)).ignoringDisable(true);
    }

    private void setActivePlayback(AnimationUDPs animationUDPs) {
        if (animationUDPs == null || animationUDPs.isEmpty()) {
            activePlayback = null;
            return;
        }

        ensurePlaybackNotifierStarted();
        activePlayback = new PlaybackState(animationUDPs);
    }

    private DatagramSocket createSocket() {
        try {
            return new DatagramSocket();
        } catch (SocketException e) {
            DriverStation.reportError("WLED socket initialization failed: " + e.getMessage(), false);
            return null;
        }
    }

    private Notifier createPlaybackNotifier() {
        Notifier notifier = new Notifier(this::sendActiveFrame);
        notifier.setName("WLED Playback");
        return notifier;
    }

    private void ensurePlaybackNotifierStarted() {
        if (playbackNotifierStarted.compareAndSet(false, true)) {
            playbackNotifier.startPeriodic(NOTIFIER_INTERVAL_SECONDS);
        }
    }

    private void sendActiveFrame() {
        PlaybackState playbackState = activePlayback;
        if (socket == null || playbackState == null) {
            return;
        }
        AnimationUDPs animation = playbackState.animation;
        if (animation.isEmpty()) {
            return;
        }

        try {
            if (playbackState.currentFrameIndex != playbackState.lastSentFrameIndex) {
                animation.sendFrame(socket, playbackState.currentFrameIndex);
                playbackState.lastSentFrameIndex = playbackState.currentFrameIndex;
            }
        } catch (IOException e) {
            DriverStation.reportError("WLED packet send failed: " + e.getMessage(), false);
            return;
        }

        double frameDurationSeconds = animation.getFrameDisplayDurationSeconds(playbackState.currentFrameIndex);
        double nowSeconds = Timer.getFPGATimestamp();
        if (frameDurationSeconds <= 0.0
                || nowSeconds - playbackState.frameStartTimeSeconds >= frameDurationSeconds) {
            playbackState.currentFrameIndex++;
            if (playbackState.currentFrameIndex >= animation.getFrameCount()) {
                playbackState.currentFrameIndex = 0;
            }
            playbackState.frameStartTimeSeconds = nowSeconds;
        }
    }
}
