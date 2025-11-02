using System;

namespace eeprom_programmer;

public class SerialClientException : Exception
{
    public SerialClientException(string message)
        : base(message)
    {
    }

    public SerialClientException(string message, Exception innerException)
        : base(message, innerException)
    {
    }
}