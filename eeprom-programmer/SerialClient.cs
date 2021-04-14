﻿using System;
using System.IO;
using System.IO.Ports;
using System.Linq;

namespace eeprom_programmer
{
    public class SerialClient : IDisposable
    {
        public const string DefaultPortName = "/dev/tty.usbmodem101";

        private const int DefaultBaudRate = 115200;
        private const int MaxAttempts = 5;

        private readonly SerialPort _serialPort;
        private bool _isDisposed;

        public SerialClient()
            : this(DefaultPortName, DefaultBaudRate)
        {
        }

        public SerialClient(string portName, int baudRate = DefaultBaudRate)
        {
            _serialPort = new SerialPort(portName ?? DefaultPortName, baudRate)
            {
                ReadTimeout = 1000,
                WriteTimeout = 1000
            };
        }

        public void Open()
        {
            _serialPort.Open();
        }

        public void Close()
        {
            _serialPort.Close();
        }

        public TextWriter TextWriter { get; set; }

        public string PortName
        {
            get => _serialPort.PortName;
            set => _serialPort.PortName = value;
        }

        public int BaudRate
        {
            get => _serialPort.BaudRate;
            set => _serialPort.BaudRate = value;
        }

        public int Timeout
        {
            get => _serialPort.ReadTimeout;
            set
            {
                _serialPort.ReadTimeout = value;
                _serialPort.WriteTimeout = value;
            }
        }

        public void UploadImage(BinaryImage image, int? address = null)
        {
            if (image is null)
                throw new ArgumentNullException(nameof(image));

            if (image.Length == 0)
                return;

            address ??= image.LoadAddress;

            if (address is null)
                throw new ArgumentNullException(nameof(address));

            if (address < 0 || address + image.Length > 0xffff)
                throw new ArgumentOutOfRangeException(nameof(address));

            for (int i = 0; i < image.Length; i += 32)
            {
                int pageLength = 32;

                if (image.Length - i < pageLength)
                    pageLength = image.Length - i;

                var commandResult = SendCommand($"writeprom 0x{image.LoadAddress + i:X4} {ToHex(image.Data, i, pageLength)}");

                if (!IsSuccessCode(commandResult))
                    throw new SerialClientException($"The programmer response code did not indicate success.");
            }
        }

        public BinaryImage DownloadImage(int address, int length)
        {
            if (address < 0 || address > 0xffff)
                throw new ArgumentOutOfRangeException(nameof(address));

            if (length <= 0 || address + length > 0xffff)
                throw new ArgumentOutOfRangeException(nameof(length));

            var commandResult = SendCommand($"readprom 0x{address:X4} {length}");

            if (!IsSuccessCode(commandResult))
                throw new SerialClientException($"The programmer response code did not indicate success.");

            var result = new BinaryImage(address, length);

            for (int i = 0; i < length; i += 16)
            {
                int pageLength = 16;

                if (length - i < pageLength)
                    pageLength = length - i;

                var pageString = _serialPort.ReadLine();

                TextWriter?.WriteLine(pageString);

                var parts = pageString.Split(new char[] { ' ', ':', '\r', '\n', '\t' }, StringSplitOptions.RemoveEmptyEntries);

                if (parts.Length < pageLength + 1 || parts.Length > 17)
                    throw new SerialClientException($"The programmer responded unexpectedly. The response was: {pageString}");

                int pageAddress = Convert.ToInt32(parts[0], 16);

                if (pageAddress != address + i)
                    throw new SerialClientException($"The programmer did not responded at the expected address of ${address + i:x4}. The response was: {pageString}");

                for (int j = 0; j < pageLength; j++)
                {
                    int b = Convert.ToInt32(parts[j + 1], 16);

                    if (b < 0 || b > 255)
                        throw new SerialClientException($"The programmer did not responded with a single byte. The response was: {pageString}");

                    result[i + j] = (byte)b;
                }
            }

            return result;
        }

        public bool Reset()
        {
            return IsSuccessCode(SendCommand("reset"));
        }

        private int SendCommand(string command)
        {
            WaitForCommandPrompt();

            _serialPort.WriteLine(command);

            var echo = _serialPort.ReadLine().Trim('\r');

            TextWriter?.WriteLine(echo);

            if (echo != command)
                throw new SerialClientException("The programmer didn't echo the command correctly");

            var response = _serialPort.ReadLine().Trim('\r');

            TextWriter?.WriteLine(response);

            if (string.IsNullOrWhiteSpace(response))
                throw new SerialClientException("The programmer responded with an empty line");

            var firstSpace = response.IndexOf(" ");

            if (firstSpace > 0)
            {
                var code = response.Substring(0, firstSpace);

                if (code.All(char.IsDigit) && int.TryParse(code, out var result) && 100 <= result && result < 600)
                    return result;
            }

            throw new SerialClientException("The programmer did not respond with an expected response code.");
        }

        private void WaitForCommandPrompt()
        {
            for (int attempt = 0; ; attempt++)
            {
                try
                {
                    do
                    {
                        var input = _serialPort.ReadTo("> ");

                        TextWriter?.Write(input);
                        TextWriter?.Write("> ");
                    }
                    while (_serialPort.BytesToRead > 0);

                    return;
                }
                catch (TimeoutException ex)
                {
                    if (attempt == MaxAttempts)
                        throw new SerialClientException(
                            "Timed out while waiting for a command prompt from the programmer.", ex);
                        
                    _serialPort.WriteLine(string.Empty);
                }
            }
        }

        private static bool IsSuccessCode(int responseCode)
        {
            return 200 <= responseCode && responseCode <= 299;
        }

        private static string ToHex(byte[] bytes, int startAt = 0, int length = -1)
        {
            if (length == -1)
                length = bytes.Length - startAt;

            char[] c = new char[length * 2];

            for (int i = 0; i < length; i++)
            {
                int nibble = bytes[startAt + i] >> 4;

                c[i * 2] = (char)(55 + nibble + (((nibble - 10) >> 31) & -7));

                nibble = bytes[startAt + i] & 0xF;
                c[i * 2 + 1] = (char)(55 + nibble + (((nibble - 10) >> 31) & -7));
            }

            return new string(c);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_isDisposed)
                return;

            if (disposing)
                _serialPort.Dispose();

            _isDisposed = true;
        }

        public void Dispose()
        {
            Dispose(true);

            GC.SuppressFinalize(this);
        }
    }
}