package frc.kauaibots.subsystems.wled;

import java.awt.image.BufferedImage;
import java.io.IOException;

public final class ImageAnimator implements Animator {
    private final AnimationSupport support;

    public ImageAnimator(WLEDConfig config) {
        this.support = new AnimationSupport(config);
    }

    @Override
    public AnimationUDPs animate(String path) {
        try {
            BufferedImage image = support.readImage(path);
            byte[][][] imageData = support.getImageData(image);
            return support.buildSingleFramePlayback(support.extractClippedFrame(imageData, 0));
        } catch (BadImageFormatException | IOException e) {
            return support.reportAnimationFailure("image", path, e);
        }
    }
}
