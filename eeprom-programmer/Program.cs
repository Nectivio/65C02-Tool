using System;
using System.IO;
using System.Runtime.Serialization;

namespace eeprom_programmer
{
    public class Program
    {
        public static void Main(string[] args)
        {
            try
            {
                if (args == null || args.Length == 0)
                    throw new ApplicationException("Filename was not specified");

                string fileName = args[0];

                if (fileName.StartsWith("~/"))
                {
                    string personal = Environment.GetFolderPath(Environment.SpecialFolder.Personal);

                    fileName = Path.Combine(personal, fileName[2..]);
                }

                var image = LoadBinaryImage(fileName, BinaryImageFormat.SBIN);

                string port = args.Length > 1 ? args[1] : SerialClient.DefaultPortName;
                
                using var serialClient = new SerialClient(port);

                serialClient.TextWriter = Console.Out;

                serialClient.Open();

                serialClient.UploadImage(image);

                var verifyImage = serialClient
                    .DownloadImage(image.LoadAddress.Value, image.Length);

                if (!image.Equals(verifyImage))
                    throw new ApplicationException("Image verification failed");

                serialClient.Reset();

                serialClient.Close();
            }
            catch (SerializationException e)
            {
                Console.Error.WriteLine($"ERROR reading image file: {e.Message}");
            }
            catch (SerialClientException e)
            {
                Console.Error.WriteLine($"ERROR interacting with the EEPROM programmer: {e.Message}");
            }
            catch (Exception e)
            {
                Console.Error.WriteLine($"ERROR {e.Message}");
            }
        }

        private static BinaryImage LoadBinaryImage(string fileName, BinaryImageFormat format)
        {
            var serializer = new BinaryImageSerializer
            {
                Format = format
            };

            using var stream = File.Open(fileName, FileMode.Open, FileAccess.Read);

            return serializer.Deserialize(stream);
        }
    }
}
