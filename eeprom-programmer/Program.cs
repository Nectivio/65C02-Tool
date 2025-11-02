using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.Serialization;
using CommandLine;
using CommandLine.Text;

namespace eeprom_programmer;

public static class Program
{
    public static int Main(string[] args)
    {
        var parser = new Parser(x => x.CaseInsensitiveEnumValues = true);

        var parserResult = parser.ParseArguments<CommandLineOptions>(args);

        return parserResult.MapResult(Execute, errs => DisplayHelp(parserResult, errs));
    }

    private static int Execute(CommandLineOptions options)
    {
        try
        {
            if (!TryParseAddress(options.Address, out var address))
                return 1;

            var image = LoadImage(options.Path, options.Format);

            using var serialClient = new SerialClient(options.PortName, options.BaudRate);

            if (options.Echo)
                serialClient.Out = Console.Out;

            serialClient.Open();

            if (!WriteImage(serialClient, image, address))
                return 2;

            if (!options.NoVerify && !VerifyImage(serialClient, image, address))
                return 3;

            if (options.Reset)
            {
                Console.WriteLine("Resetting target computer...");

                serialClient.Reset();
            }

            Console.WriteLine("Update completed successfully.");

            serialClient.Close();

            return 0;
        }
        catch (SerializationException e)
        {
            Console.Error.WriteLine($"ERROR reading image file: {e.Message}");

            return 4;
        }
        catch (SerialClientException e)
        {
            Console.Error.WriteLine($"ERROR interacting with the EEPROM programmer: {e.Message}");

            return 5;
        }
        catch (Exception e)
        {
            Console.Error.WriteLine($"ERROR {e.Message}");

            return 6;
        }
    }

    private static int DisplayHelp<T>(ParserResult<T> result, IEnumerable<Error> errors)
    {  
        var helpText = HelpText.AutoBuild(result, h =>
        {
            h.AutoVersion = false;
            h.Heading = "eeprom-programmer";
            h.Copyright = "Copyright (c) 2025 Nectivio Inc.";
            return HelpText.DefaultParsingErrorsHandler(result, h);
        }, e => e);

        errors.Output().WriteLine(helpText);

        return 1;
    }

    private static BinaryImage LoadImage(string filePath, BinaryImageFormat format)
    {
        if (filePath.StartsWith("~/"))
        {
            var personal = Environment.GetFolderPath(Environment.SpecialFolder.Personal);

            filePath = Path.Combine(personal, filePath[2..]);
        }

        if (format == BinaryImageFormat.Unspecified)
        {
            var extension = Path.GetExtension(filePath)
                .TrimStart('.').Replace('-', '_');
                               
            if (!Enum.TryParse(extension, true, out format))
                format = BinaryImageFormat.RAW; 
        }

        Console.WriteLine($"Loading binary image file {filePath}...");

        var serializer = new BinaryImageSerializer(format);

        using var stream = File.Open(filePath, FileMode.Open, FileAccess.Read);

        return serializer.Deserialize(stream);
    }

    private static bool WriteImage(SerialClient serialClient, BinaryImage image, ushort? address)
    {
        try
        {
            Console.WriteLine("Uploading ROM Image...");

            serialClient.UploadImage(image, address);

            return true;
        }
        catch (ArgumentNullException ex) when (ex.ParamName == "address")
        {
            Console.Error.WriteLine(
                "Unable to upload ROM image, the load address was not specified either in the image file or on the command line.");
        }

        return false;
    }

    private static bool VerifyImage(SerialClient serialClient, BinaryImage image, ushort? address)
    {
        Console.WriteLine("Verifying ROM Image...");

        var verifyImage = serialClient
            .DownloadImage(address ?? image.LoadAddress!.Value, image.Length);

        if (image.Equals(verifyImage))
            return true;
        
        Console.Error.WriteLine("ERROR: Image verification failed. Is the ROM Write Locked?");
        
        return false;
    }

    private static bool TryParseAddress(string input, out ushort? address)
    {
        address = null;

        try
        {
            address = ParseUInt16(input);

            return true;
        }
        catch (FormatException)
        {
            Console.Error.WriteLine($"The address '{input}' is not a valid address.");
        }
        catch (OverflowException)
        {
            Console.Error.WriteLine($"The address '{input}' must be in the range 0x0000 to 0xFFFF");
        }

        return false;
    }

    private static ushort? ParseUInt16(string input)
    {
        if (string.IsNullOrWhiteSpace(input))
            return null;     
     
        if (input.StartsWith('$'))
            return ushort.Parse(input[1..], NumberStyles.HexNumber);

        if (input.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
            return ushort.Parse(input[2..], NumberStyles.HexNumber);

        return ushort.Parse(input, NumberStyles.AllowLeadingWhite | NumberStyles.AllowTrailingWhite);
    }
}