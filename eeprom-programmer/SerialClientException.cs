using System;
using System.Runtime.Serialization;

namespace eeprom_programmer
{
    [Serializable]
    public class SerialClientException : Exception
    {
        public SerialClientException()
        {
        }

        public SerialClientException(string message)
            : base(message)
        {
        }

        public SerialClientException(string message, Exception innerException)
            : base(message, innerException)
        {
        }

        protected SerialClientException(SerializationInfo info, StreamingContext context) : base(info, context)
        {
        }
    }
}