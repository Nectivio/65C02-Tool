using System;

namespace eeprom_programmer;

public class BinaryImage : IEquatable<BinaryImage>
{
    private byte[] _data;

    public BinaryImage()
    {
    }

    public BinaryImage(int loadAddress, int length = 0)
    {
        if (loadAddress is < 0 or > 0xffff)
            throw new ArgumentOutOfRangeException(nameof(loadAddress));

        if (length < 0 || loadAddress + length - 1 > 0xffff)
            throw new ArgumentOutOfRangeException(nameof(length));

        LoadAddress = (ushort) loadAddress;

        if (length > 0)
            _data = new byte[length];
    }

    public ushort? LoadAddress { get; set; }

    public byte[] Data
    {
        get => _data;
        set
        {
            if (_data?.Length > 0xFFF)
                throw new ArgumentException("Data can not exceed 64 KiB in length");

            _data = value;
        }
    }

    public ushort Length => (ushort)(_data?.Length ?? 0);
 
    public byte this[int index]
    {
        get
        {
            if (index < 0 || index >= Length)
                throw new ArgumentOutOfRangeException(nameof(index));

            return _data[index];
        }
        set
        {
            if (index < 0 || index >= Length)
                throw new ArgumentOutOfRangeException(nameof(index));

            _data[index] = value;
        }
    }

    public bool Equals(BinaryImage other)
    {
        if (other == null)
            return false;

        if (Length != other.Length)
            return false;

        int length = Length;

        for (var i = 0; i < length; i++)
        {
            if (_data[i] != other._data[i])
                return false;
        }

        return true;
    }

    public override bool Equals(object obj)
    {
        return Equals(obj as BinaryImage);
    }

    public override int GetHashCode()
    {
        // ReSharper disable NonReadonlyMemberInGetHashCode
        return (LoadAddress?.GetHashCode() ?? 0) ^ (_data?.GetHashCode() ?? 0);
    }
}