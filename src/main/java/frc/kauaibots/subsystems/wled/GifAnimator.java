package frc.kauaibots.subsystems.wled;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;

import com.madgag.gif.fmsware.GifDecoder;

public final class GifAnimator implements Animator {
    private final AnimationSupport support;

    public GifAnimator(WLEDConfig config) {
        this.support = new AnimationSupport(config);
    }

    @Override
    public AnimationUDPs animate(String path) {
        File gifFile = new File(path);
        try (InputStream inputStream = gifFile.toURI().toURL().openStream()) {
            GifDecoder gifDecoder = new GifDecoder();
            int status = gifDecoder.read(inputStream);
            if (status == GifDecoder.STATUS_OPEN_ERROR) {
                throw new BadImageFormatException("GIF file could not be opened.");
            }
            if (status != GifDecoder.STATUS_OK) {
                throw new BadImageFormatException("GIF could not be decoded.");
            }

            int frameCount = gifDecoder.getFrameCount();
            if (frameCount <= 0) {
                throw new BadImageFormatException("GIF contains no frames.");
            }

            support.validateGifDimensions(gifDecoder.getFrame(0), "GIF logical screen");
            return support.buildPlaybackPackets(frameCount, frameIndex -> support.buildGifFrame(gifDecoder, frameIndex));
        } catch (BadImageFormatException | IOException e) {
            return support.reportAnimationFailure("gif", path, e);
        }
    }
}
