using System;
using System.IO;
using System.IO.Ports;
using System.Threading;

namespace eeprom_programmer
{
    public class Program
    {
        public static void Main(string[] args)
        {
            if (args == null || args.Length == 0)
                throw new ApplicationException("no file name");

            string fileName = args[0];

            if (fileName.StartsWith("~/"))
            {
                string personal = Environment.GetFolderPath(Environment.SpecialFolder.Personal);
                fileName = Path.Combine(personal, fileName.Substring(2));
            }

            string port = "/dev/tty.usbmodem101";

            if (args.Length > 1)
                port = args[1];

            using var stream = File.Open(fileName, FileMode.Open, FileAccess.Read);
            using var serialPort = new SerialPort(port, 115200);

            serialPort.ReadTimeout = 1000;
            serialPort.WriteTimeout = 1000;
            
            byte[] header = new byte[4];

            stream.Read(header, 0, header.Length);

            int origin = (header[1] << 8) + header[0];
            int length = (header[3] << 8) + header[2];

            Console.WriteLine($"Origin: ${origin:X}, Length {length} bytes");
            
            byte[] data = new byte[length];

            var bytesRead = stream.Read(data, 0, length);

            serialPort.Open();

            for (int i = 0; i < 100 && serialPort.BytesToRead == 0; i++)
              Thread.Sleep(100);

            serialPort.WriteLine("");
 
            for (int i = 0; i < length; i +=  32)
            {
                int pageLength = 32;

                if (length - i < pageLength)
                  pageLength = length - i;
                
                if (!WaitForPrompt(serialPort))
                    throw new ApplicationException("Didn't get command prompt.");
                
                var command = $"writeprom 0x{(origin + i):X4} {ToHex(data, i, pageLength)}";
                serialPort.WriteLine(command);
            }

            WaitForPrompt(serialPort);
            serialPort.WriteLine($"readprom 0x{origin:X4} {length}");

            WaitForPrompt(serialPort);
            serialPort.WriteLine("reset");

            WaitForPrompt(serialPort);
            serialPort.Close();
        }

        private static string ToHex(byte[] bytes, int startAt = 0, int length = -1)
        {
            if (length == -1)
                length = bytes.Length - startAt;
            
            char[] c = new char[length * 2];
            
            for (int i = 0; i < length; i++)
            {
                int nibble = bytes[startAt + i] >> 4;

                c[i * 2] = (char)(55 + nibble + (((nibble-10)>>31)&-7));
                
                nibble = bytes[startAt + i] & 0xF;
                c[i * 2 + 1] = (char)(55 + nibble + (((nibble-10)>>31)&-7));
            }
            
            return new string(c);
        }

        private static bool WaitForPrompt(SerialPort serialPort)
        {
            for (int attempt = 0; attempt <= 5; attempt++)
            {
                try
                {
                    
                    do
                    {
                        var input = serialPort.ReadTo("> ");

                        Console.Write(input);
                        Console.Write("> ");
                    }
                    while (serialPort.BytesToRead > 0);

                    return true;
                }
                catch (TimeoutException)
                {
                    serialPort.WriteLine("");
                }
            }

            return false;
        }
    }
}
