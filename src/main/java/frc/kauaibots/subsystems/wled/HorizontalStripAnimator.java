package frc.kauaibots.subsystems.wled;

import java.awt.image.BufferedImage;
import java.io.IOException;

public final class HorizontalStripAnimator implements Animator {
    private final AnimationSupport support;
    private final WLEDConfig config;

    public HorizontalStripAnimator(WLEDConfig config) {
        this.config = config;
        this.support = new AnimationSupport(config);
    }

    @Override
    public AnimationUDPs animate(String path) {
        try {
            BufferedImage image = support.readImage(path);
            int realWidth = image.getWidth();
            if (realWidth % config.width() != 0) {
                throw new BadImageFormatException(
                        "Animation width must be an integer multiple of " + config.width() + " pixels.");
            }

            byte[][][] imageData = support.getImageData(image);
            int frameCount = realWidth / config.width();
            return support.buildPlaybackPackets(
                    frameCount,
                    frameIndex -> new AnimationSupport.FrameData(
                            support.extractClippedFrame(imageData, frameIndex * config.width()),
                            0.0));
        } catch (BadImageFormatException | IOException e) {
            return support.reportAnimationFailure("animation", path, e);
        }
    }
}
