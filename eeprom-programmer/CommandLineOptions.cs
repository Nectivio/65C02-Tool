using CommandLine;

namespace eeprom_programmer
{
    public class CommandLineOptions
    {
        [Value(0, MetaName = "Path", Required = true, HelpText = "Path to an eeprom image file.")]
        public string Path { get; set; }

        [Option('a', HelpText = "Start Address to load the image file at")]
        public string Address { get; set; }

        [Option('b', HelpText = "Baud Rate", Default = 115200)]
        public int BaudRate { get; set; }

        [Option('e', HelpText = "Echo the communication between the computer and the EEPROM programmer to the console")]
        public bool Echo { get; set; }

        [Option('f', HelpText = "The format of the binary image file", Default = BinaryImageFormat.Unspecified)]
        public BinaryImageFormat Format { get; set; }

        [Option('p', HelpText = "Serial Port")]
        public string PortName { get; set; }

        [Option(HelpText = "Skip automatic verification of the uploaded image")]
        public bool NoVerify { get; set; }

        [Option('r', HelpText = "Reset the target computer on successful upload")]
        public bool Reset { get; set; }
    }
}
