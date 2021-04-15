using System;
using System.IO;
using System.Runtime.Serialization;

namespace eeprom_programmer
{
    public class BinaryImageSerializer
    {
        public BinaryImageSerializer()
        {
        }

        public BinaryImageSerializer(BinaryImageFormat format)
        {
            Format = format;
        }

        public BinaryImageFormat Format { get; set; }

        public void Serialize(Stream stream, BinaryImage image)
        {
            if (Format == BinaryImageFormat.PRG || Format == BinaryImageFormat.SBIN)
            {
                if (image.LoadAddress == null)
                    throw new SerializationException("The binary image doesn't not have a load address.");

                WriteUInt16(stream, image.LoadAddress.Value);

                if (Format == BinaryImageFormat.SBIN)
                    WriteUInt16(stream, image.Length);
            }

            if (image.Length > 0)
            {
                stream.Write(image.Data, 0, image.Length);
            }
        }

        public BinaryImage Deserialize(Stream stream)
        {
            var result = new BinaryImage();

            int expectedLength = -1;

            if (Format == BinaryImageFormat.PRG || Format == BinaryImageFormat.SBIN)
                result.LoadAddress = ReadUInt16(stream);

            if (Format == BinaryImageFormat.SBIN)
                expectedLength = ReadUInt16(stream);

            var buffer = new byte[65536];

            var bytesRead = stream.Read(buffer, 0, buffer.Length);

            if (expectedLength >= 0 && bytesRead != expectedLength)
                throw new SerializationException("The stream length did not match the length in the header.");

            result.Data = new byte[bytesRead];

            Array.Copy(buffer, result.Data, bytesRead);

            return result;
        }

        private static ushort ReadUInt16(Stream stream)
        {
            var result = (ushort)stream.ReadByte();
            result |= (ushort)(stream.ReadByte() << 8);
            return result;
        }

        private static void WriteUInt16(Stream stream, ushort word)
        {
            stream.WriteByte((byte)(word & 0xFF));
            stream.WriteByte((byte)(word >> 8));
        }
    }
}
