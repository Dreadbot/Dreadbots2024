package util.misc;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class VisionPosition implements StructSerializable {

    public final double x;
    public final double y;
    public final int ID;
    public final long frameIdx;

    public VisionPosition(double x, double y, int ID, long frameIdx) {
        this.x = x;
        this.y = y;
        this.ID = ID;
        this.frameIdx = frameIdx;
    }

    public static final VisionPositionStruct struct = new VisionPositionStruct();


    public static class VisionPositionStruct implements Struct<VisionPosition> {

        @Override
        public Class<VisionPosition> getTypeClass() {
            return VisionPosition.class;
        }

        @Override
        public String getTypeString() {
            return "struct:position";
        }

        @Override
        public int getSize() {
           return kSizeDouble * 2 + kSizeInt32 + kSizeInt64;
        }

        @Override
        public String getSchema() {
            return "double x;double y;int ID";
        }

        @Override
        public VisionPosition unpack(ByteBuffer bb) {
            double x = bb.getDouble();
            double y = bb.getDouble();
            int ID = bb.getInt();
            long frameIdx = bb.getLong();
            return new VisionPosition(x, y, ID, frameIdx);
        }

        @Override
        public void pack(ByteBuffer bb, VisionPosition value) {
            bb.putDouble(value.x);
            bb.putDouble(value.y);
            bb.putInt(value.ID);
            bb.putLong(value.frameIdx);
        }

    }
}
