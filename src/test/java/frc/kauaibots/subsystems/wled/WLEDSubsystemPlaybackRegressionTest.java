package frc.kauaibots.subsystems.wled;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.Base64;
import java.util.Iterator;
import java.util.Locale;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageTypeSpecifier;
import javax.imageio.ImageWriteParam;
import javax.imageio.ImageWriter;
import javax.imageio.metadata.IIOMetadata;
import javax.imageio.metadata.IIOMetadataNode;
import javax.imageio.stream.ImageOutputStream;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class WLEDSubsystemPlaybackRegressionTest {
    private static final WLEDConfig TEST_CONFIG = new WLEDConfig("127.0.0.1", 4048, 32, 16, 8);

    @TempDir
    Path tempDir;

    @Test
    void displayBitmapMatchesSnapshot() throws IOException {
        ImageAnimator imageAnimator = new ImageAnimator(TEST_CONFIG);

        assertPlaybackMatchesSnapshot(
                "wled-prepare-image.snapshot.txt",
                imageAnimator.animate(writeImage("image.png", createImageFixture()).toString()));
    }

    @Test
    void animateBitmapAsMarqueeMatchesSnapshot() throws IOException {
        MarqueeAnimator marqueeAnimator = new MarqueeAnimator(TEST_CONFIG);

        assertPlaybackMatchesSnapshot(
                "wled-prepare-marquee.snapshot.txt",
                marqueeAnimator.animate(writeImage("marquee.png", createMarqueeFixture()).toString()));
    }

    @Test
    void animateFromBitmapAnimationStripMatchesSnapshot() throws IOException {
        HorizontalStripAnimator horizontalStripAnimator = new HorizontalStripAnimator(TEST_CONFIG);

        assertPlaybackMatchesSnapshot(
                "wled-prepare-strip.snapshot.txt",
                horizontalStripAnimator.animate(writeImage("strip.png", createStripFixture()).toString()));
    }

    @Test
    void animateGifMatchesSnapshot() throws IOException {
        GifAnimator gifAnimator = new GifAnimator(TEST_CONFIG);

        assertPlaybackMatchesSnapshot(
                "wled-prepare-gif.snapshot.txt",
                gifAnimator.animate(writeGif("fixture.gif", createGifFrames(), new int[] {5, 9}).toString()));
    }

    private void assertPlaybackMatchesSnapshot(String resourceName, AnimationUDPs playback)
            throws IOException {
        assertEquals(loadSnapshot(resourceName), snapshot(playback));
    }

    private String loadSnapshot(String resourceName) throws IOException {
        try (InputStream inputStream = getClass().getResourceAsStream(resourceName)) {
            assertNotNull(inputStream, "Missing snapshot resource: " + resourceName);
            return new String(inputStream.readAllBytes(), StandardCharsets.UTF_8).stripTrailing();
        }
    }

    private Path writeImage(String fileName, BufferedImage image) throws IOException {
        Path imagePath = tempDir.resolve(fileName);
        ImageIO.write(image, "png", imagePath.toFile());
        return imagePath;
    }

    private Path writeGif(String fileName, BufferedImage[] frames, int[] delaysCentiseconds) throws IOException {
        Path gifPath = tempDir.resolve(fileName);
        Iterator<ImageWriter> writers = ImageIO.getImageWritersBySuffix("gif");
        ImageWriter writer = writers.next();
        ImageWriteParam params = writer.getDefaultWriteParam();
        ImageTypeSpecifier imageType = ImageTypeSpecifier.createFromBufferedImageType(BufferedImage.TYPE_INT_ARGB);

        try (ImageOutputStream output = ImageIO.createImageOutputStream(gifPath.toFile())) {
            writer.setOutput(output);
            writer.prepareWriteSequence(null);

            for (int i = 0; i < frames.length; i++) {
                IIOMetadata metadata = writer.getDefaultImageMetadata(imageType, params);
                String metadataFormat = metadata.getNativeMetadataFormatName();
                IIOMetadataNode root = (IIOMetadataNode) metadata.getAsTree(metadataFormat);

                IIOMetadataNode graphicsControlExtension = getOrCreateChild(root, "GraphicControlExtension");
                graphicsControlExtension.setAttribute("disposalMethod", "none");
                graphicsControlExtension.setAttribute("userInputFlag", "FALSE");
                graphicsControlExtension.setAttribute("transparentColorFlag", "FALSE");
                graphicsControlExtension.setAttribute("delayTime", Integer.toString(delaysCentiseconds[i]));
                graphicsControlExtension.setAttribute("transparentColorIndex", "0");

                IIOMetadataNode imageDescriptor = getOrCreateChild(root, "ImageDescriptor");
                imageDescriptor.setAttribute("imageLeftPosition", "0");
                imageDescriptor.setAttribute("imageTopPosition", "0");
                imageDescriptor.setAttribute("imageWidth", Integer.toString(frames[i].getWidth()));
                imageDescriptor.setAttribute("imageHeight", Integer.toString(frames[i].getHeight()));
                imageDescriptor.setAttribute("interlaceFlag", "FALSE");

                metadata.setFromTree(metadataFormat, root);
                writer.writeToSequence(new IIOImage(frames[i], null, metadata), params);
            }

            writer.endWriteSequence();
        } finally {
            writer.dispose();
        }

        return gifPath;
    }

    private static IIOMetadataNode getOrCreateChild(IIOMetadataNode parent, String childName) {
        for (int i = 0; i < parent.getLength(); i++) {
            if (parent.item(i) instanceof IIOMetadataNode child && childName.equals(child.getNodeName())) {
                return child;
            }
        }

        IIOMetadataNode child = new IIOMetadataNode(childName);
        parent.appendChild(child);
        return child;
    }

    private static BufferedImage createImageFixture() {
        BufferedImage image = new BufferedImage(TEST_CONFIG.width(), TEST_CONFIG.height(), BufferedImage.TYPE_INT_ARGB);
        fillWithPattern(image, 3, 5, 7);
        return image;
    }

    private static BufferedImage createMarqueeFixture() {
        BufferedImage image = new BufferedImage(5, TEST_CONFIG.height(), BufferedImage.TYPE_INT_ARGB);
        fillWithPattern(image, 11, 13, 17);
        return image;
    }

    private static BufferedImage createStripFixture() {
        BufferedImage image =
                new BufferedImage(TEST_CONFIG.width() * 2, TEST_CONFIG.height(), BufferedImage.TYPE_INT_ARGB);
        fillWithPattern(image, 19, 23, 29);
        return image;
    }

    private static BufferedImage[] createGifFrames() {
        BufferedImage firstFrame =
                new BufferedImage(TEST_CONFIG.width(), TEST_CONFIG.height(), BufferedImage.TYPE_INT_ARGB);
        BufferedImage secondFrame =
                new BufferedImage(TEST_CONFIG.width(), TEST_CONFIG.height(), BufferedImage.TYPE_INT_ARGB);
        fillWithPattern(firstFrame, 31, 37, 41);
        fillWithPattern(secondFrame, 43, 47, 53);
        return new BufferedImage[] {firstFrame, secondFrame};
    }

    private static void fillWithPattern(BufferedImage image, int redFactor, int greenFactor, int blueFactor) {
        for (int y = 0; y < image.getHeight(); y++) {
            for (int x = 0; x < image.getWidth(); x++) {
                int red = (x * redFactor + y * 7) & 0xFF;
                int green = (x * 5 + y * greenFactor) & 0xFF;
                int blue = (x * blueFactor + y * 3) & 0xFF;
                image.setRGB(x, y, new Color(red, green, blue).getRGB());
            }
        }
    }

    private static String snapshot(AnimationUDPs playback) {
        StringBuilder builder = new StringBuilder();
        builder.append("frameCount=").append(playback.getFrameCount()).append('\n');
        builder.append("packetsPerFrame=").append(playback.getPacketsPerFrame()).append('\n');
        builder.append("packetCount=").append(playback.getPacketCount()).append('\n');
        builder.append("durations=");
        for (int i = 0; i < playback.getFrameCount(); i++) {
            if (i > 0) {
                builder.append(',');
            }
            builder.append(String.format(Locale.ROOT, "%.2f", playback.getFrameDisplayDurationSeconds(i)));
        }
        builder.append('\n');
        for (int i = 0; i < playback.getPacketCount(); i++) {
            builder.append("packet[").append(i).append("]=")
                    .append(Base64.getEncoder().encodeToString(playback.getPacketDataCopy(i)))
                    .append('\n');
        }
        return builder.toString().trim();
    }
}
