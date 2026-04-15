package frc.kauaibots.subsystems.wled;

import java.awt.image.BufferedImage;
import java.io.IOException;

public final class MarqueeAnimator implements Animator {
    private final AnimationSupport support;

    public MarqueeAnimator(WLEDConfig config) {
        this.support = new AnimationSupport(config);
    }

    @Override
    public AnimationUDPs animate(String path) {
        try {
            BufferedImage image = support.readImage(path);
            byte[][][] imageData = support.getImageData(image);
            int frameCount = image.getWidth();
            return support.buildPlaybackPackets(
                    frameCount,
                    frameIndex -> new AnimationSupport.FrameData(
                            support.extractWrappedFrame(imageData, frameIndex),
                            0.0));
        } catch (BadImageFormatException | IOException e) {
            return support.reportAnimationFailure("marquee", path, e);
        }
    }
}
