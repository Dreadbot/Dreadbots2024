package util.misc;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class VisionPosition implements StructSerializable {

    public final double x;
    public final double y;
    public final int ID;

    public VisionPosition(double x, double y, int ID) {
        this.x = x;
        this.y = y;
        this.ID = ID;
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
           return kSizeDouble * 2 + kSizeInt32;
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
            return new VisionPosition(x, y, ID);
        }

        @Override
        public void pack(ByteBuffer bb, VisionPosition value) {
            bb.putDouble(value.x);
            bb.putDouble(value.y);
            bb.putInt(value.ID);
        }

    }
}
