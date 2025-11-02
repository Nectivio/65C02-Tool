namespace eeprom_programmer;

public enum BinaryImageFormat
{
    Unspecified,

    BIN,    // Raw Binary image

    PRG,    // 2-Byte header for the defining the load address

    SBIN,   // 4-byte header for load address, and image length

    // Synonyms

    RAW = BIN,          // Raw Binary image

    CBM_PRG = PRG,      // Commodore PRG, 

    APPLE_BIN = SBIN,   // Apple DOS 3.3 binary file header
}