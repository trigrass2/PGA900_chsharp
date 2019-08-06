using System;
using System.Text;
using System.Runtime.InteropServices;

using FTD2XX_NET;

namespace CSharp_FT4222_SPI_Master
{
    class Constants
    {
            static public byte wLSB_upper                  = 0x10;
            static public byte wLSB_lower                  = 0xf0;
            static public byte wLSB_firstbyte = 0x01;

            static public byte wMSB_upper = 0x30;
            static public byte wMSB_lower = 0xf0;
            static public byte wMSB_firstbyte = 0x01;

            static public byte rTxS_firstbyte = 0x00;
            static public byte rTxS_secondbyte = 0xC0;
            static public byte rTxS_thirdbyte = 0x00;

            static public byte sTxS_firstbyte = 0x00;
            static public byte sTxS_secondbyte = 0xD0;
            static public byte sTxS_thirdbyte = 0x10;


            static public byte r2Bpga_firstbyte = 0x00;
            static public byte r2Bpga_secondbyte = 0x80;
            static public byte r2Bpga_thirdbyte = 0x00;

            static public byte CS_hold_time16Mhz = 66;

            static public UInt16 CO_TEMPERATURE         =     3800;

        public enum StateOfOperation_t
        {
            IdleState,
            Main = 0x0001,
            Configuration,
            Gasanalysis = 3,
            TemperatureConfiguration,
            Developement,
            CheckState = 0xAAAA
        }
        ;

            
public enum TemperatureConfigurationCommand_t
        {
            ReadConfigurationData = 0x0201,
            StoreSensorData,
            SetSingleParameter,
            SetBridgeVoltage,
            CheckBridgeVoltage,
            CheckConfiguration,
            CancelTemperatureConfiguration
        }
        ;


public enum ConfigurationCommand_t
        {
            SetHardware = 0x0301,
            SetRegulationParameter,
            ReadRegulationParameter,
            CancelConfiguration
        }
        ;


public enum GasanalysisCommand_t
        {
            StartGasanalysis = 0x0401,
            ReadGasanalysisData,
            StopGasanalysis,
            CancelGasanalysis,
        }
        ;


public enum TypeOfGasanalysis_t
        {
            Idle1 = 0x0500,
            ConstantVoltage,
            ConstantCurrent,
            ConstantTemperature,
            ConstantOverTemperature,
            TemperatureMeasurement,
        }
        ;


public enum TypeOfGasanalysisParameter_t
        {
            CO2 = 1,
            HeaterTemperature = 2,
            EnvironmentTemperature = 4,
            DACValueParameter = 8,
            HeaterResistance = 16,
            Rsoll = 32,
            Regulationerror = 64,
        }
        ;


public enum Case_t
        {
            Side = 0x0601,
            Top,
            Hole
        }
        ;


public enum OutputMode_t
        {
            PGA900Out = 0x0701,
            Adder,
            Amplifier
        }
        ;


public enum OperationMode_t
        {
            Pump = 0x0801,
            Clap,
            Diffusion
        }
        ;


public enum OperationMessage_t
        {
            TempSensK_SendSuccess = 0x0901,
            Temperature_Success,
            Temperature_MSB_SendSucces,
            Resistance_TSens_SendSuccess,
            Resistance_HSens_SendSuccess,
            BridgeVoltage_Success,
            Sensor_Configured,
            Sensor_NotConfigured,
            Measurement_Stopped,
            ParameterSendFailed,
            ParameterReadSuccess,
            ParameterReadFailed,
            RegulationValue_Success,
            SelectedParameter_Success
        }
        ;
    }

    class Program
    {
        //**************************************************************************
        //
        // FUNCTION IMPORTS FROM FTD2XX DLL
        //
        //**************************************************************************

        [DllImport("ftd2xx.dll")]
        static extern FTDI.FT_STATUS FT_CreateDeviceInfoList(ref UInt32 numdevs);

        [DllImport("ftd2xx.dll")]
        static extern FTDI.FT_STATUS FT_GetDeviceInfoDetail(UInt32 index, ref UInt32 flags, ref FTDI.FT_DEVICE chiptype, ref UInt32 id, ref UInt32 locid, byte[] serialnumber, byte[] description, ref IntPtr ftHandle);

        //[DllImportAttribute("ftd2xx.dll", CallingConvention = CallingConvention.Cdecl)]
        [DllImport("ftd2xx.dll")]
        static extern FTDI.FT_STATUS FT_OpenEx(uint pvArg1, int dwFlags, ref IntPtr ftHandle);

        //[DllImportAttribute("ftd2xx.dll", CallingConvention = CallingConvention.Cdecl)]
        [DllImport("ftd2xx.dll")]
        static extern FTDI.FT_STATUS FT_Close(IntPtr ftHandle);


        const byte FT_OPEN_BY_SERIAL_NUMBER = 1;
        const byte FT_OPEN_BY_DESCRIPTION = 2;
        const byte FT_OPEN_BY_LOCATION = 4;

        //**************************************************************************
        //
        // FUNCTION IMPORTS FROM LIBFT4222 DLL
        //
        //**************************************************************************

        [DllImport("LibFT4222.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern FT4222_STATUS FT4222_SetClock(IntPtr ftHandle, FT4222_ClockRate clk);

        [DllImport("LibFT4222.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern FT4222_STATUS FT4222_GetClock(IntPtr ftHandle, ref FT4222_ClockRate clk);

        [DllImport("LibFT4222.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern FT4222_STATUS FT4222_SPIMaster_Init(IntPtr ftHandle, FT4222_SPIMode ioLine, FT4222_SPIClock clock, FT4222_SPICPOL cpol, FT4222_SPICPHA cpha, Byte ssoMap);

        [DllImport("LibFT4222.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern FT4222_STATUS FT4222_SPI_SetDrivingStrength(IntPtr ftHandle, SPI_DrivingStrength clkStrength, SPI_DrivingStrength ioStrength, SPI_DrivingStrength ssoStregth);

        [DllImport("LibFT4222.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern FT4222_STATUS FT4222_SPIMaster_SingleReadWrite(IntPtr ftHandle, ref byte readBuffer, ref byte writeBuffer, ushort bufferSize, ref ushort sizeTransferred, bool isEndTransaction);

        [DllImport("LibFT4222.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern FT4222_STATUS FT4222_SPI_Reset(IntPtr ftHandle);


        // variable
        static FTDI.FT_DEVICE_INFO_NODE devInfo = new FTDI.FT_DEVICE_INFO_NODE();
        static IntPtr ftHandle = new IntPtr();
        static FTDI.FT_STATUS ftStatus = 0;
        static FT4222_STATUS ft42Status = 0;

        // FT4222 Device status
        public enum FT4222_STATUS
        {
            FT4222_OK,
            FT4222_INVALID_HANDLE,
            FT4222_DEVICE_NOT_FOUND,
            FT4222_DEVICE_NOT_OPENED,
            FT4222_IO_ERROR,
            FT4222_INSUFFICIENT_RESOURCES,
            FT4222_INVALID_PARAMETER,
            FT4222_INVALID_BAUD_RATE,
            FT4222_DEVICE_NOT_OPENED_FOR_ERASE,
            FT4222_DEVICE_NOT_OPENED_FOR_WRITE,
            FT4222_FAILED_TO_WRITE_DEVICE,
            FT4222_EEPROM_READ_FAILED,
            FT4222_EEPROM_WRITE_FAILED,
            FT4222_EEPROM_ERASE_FAILED,
            FT4222_EEPROM_NOT_PRESENT,
            FT4222_EEPROM_NOT_PROGRAMMED,
            FT4222_INVALID_ARGS,
            FT4222_NOT_SUPPORTED,
            FT4222_OTHER_ERROR,
            FT4222_DEVICE_LIST_NOT_READY,

            FT4222_DEVICE_NOT_SUPPORTED = 1000,        // FT_STATUS extending message
            FT4222_CLK_NOT_SUPPORTED,     // spi master do not support 80MHz/CLK_2
            FT4222_VENDER_CMD_NOT_SUPPORTED,
            FT4222_IS_NOT_SPI_MODE,
            FT4222_IS_NOT_I2C_MODE,
            FT4222_IS_NOT_SPI_SINGLE_MODE,
            FT4222_IS_NOT_SPI_MULTI_MODE,
            FT4222_WRONG_I2C_ADDR,
            FT4222_INVAILD_FUNCTION,
            FT4222_INVALID_POINTER,
            FT4222_EXCEEDED_MAX_TRANSFER_SIZE,
            FT4222_FAILED_TO_READ_DEVICE,
            FT4222_I2C_NOT_SUPPORTED_IN_THIS_MODE,
            FT4222_GPIO_NOT_SUPPORTED_IN_THIS_MODE,
            FT4222_GPIO_EXCEEDED_MAX_PORTNUM,
            FT4222_GPIO_WRITE_NOT_SUPPORTED,
            FT4222_GPIO_PULLUP_INVALID_IN_INPUTMODE,
            FT4222_GPIO_PULLDOWN_INVALID_IN_INPUTMODE,
            FT4222_GPIO_OPENDRAIN_INVALID_IN_OUTPUTMODE,
            FT4222_INTERRUPT_NOT_SUPPORTED,
            FT4222_GPIO_INPUT_NOT_SUPPORTED,
            FT4222_EVENT_NOT_SUPPORTED,
        };

        public enum FT4222_ClockRate
        {
            SYS_CLK_60 = 0,
            SYS_CLK_24,
            SYS_CLK_48,
            SYS_CLK_80,

        };

        public enum FT4222_SPIMode
        {
            SPI_IO_NONE = 0,
            SPI_IO_SINGLE = 1,
            SPI_IO_DUAL = 2,
            SPI_IO_QUAD = 4,

        };

        public enum FT4222_SPIClock
        {
            CLK_NONE = 0,
            CLK_DIV_2,      // 1/2   System Clock
            CLK_DIV_4,      // 1/4   System Clock
            CLK_DIV_8,      // 1/8   System Clock
            CLK_DIV_16,     // 1/16  System Clock
            CLK_DIV_32,     // 1/32  System Clock
            CLK_DIV_64,     // 1/64  System Clock
            CLK_DIV_128,    // 1/128 System Clock
            CLK_DIV_256,    // 1/256 System Clock
            CLK_DIV_512,    // 1/512 System Clock

        };

        public enum FT4222_SPICPOL
        {
            CLK_IDLE_LOW = 0,
            CLK_IDLE_HIGH = 1,
        };

        public enum FT4222_SPICPHA
        {
            CLK_LEADING = 0,
            CLK_TRAILING = 1,
        };

        public enum SPI_DrivingStrength
        {
            DS_4MA = 0,
            DS_8MA,
            DS_12MA,
            DS_16MA,
        };

        [StructLayout(LayoutKind.Explicit)]
        struct uint168_t
        {
            [FieldOffset(0)]
            public byte uintb0;
            [FieldOffset(1)]
            public byte uintb1;
            [FieldOffset(0)]
            public UInt16 uinta;
        }

        static byte Uca0SpiSendRec(byte data, bool IsEnd)
        {
            ushort sizeTransferred = 0;
            byte readByte = 0x00 ;
  
            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readByte, ref data, 1, ref sizeTransferred, IsEnd);
            if (ft42Status != FT4222_STATUS.FT4222_OK)
            {
                Console.WriteLine("Write NG: {0}", ft42Status);
            }
            Console.WriteLine("Wrote: {0:X} Read: {1:X}", data, readByte);
            return readByte;
        }

    static byte WriteByteToLSB(byte daten)
        {
            Uca0SpiSendRec(Constants.wLSB_firstbyte, false); //0x01
            Uca0SpiSendRec(BitConverter.GetBytes(Constants.wLSB_upper | (daten >> 4))[0], false); //0x10
            Uca0SpiSendRec(BitConverter.GetBytes(Constants.wLSB_lower & (daten << 4))[0], true); //0xf0

            return 0;
        }


        static byte WriteByteToMSB(byte daten)
        {
            Uca0SpiSendRec(Constants.wMSB_firstbyte, false);
            Uca0SpiSendRec(BitConverter.GetBytes(Constants.wMSB_upper | (daten >> 4))[0], false); //0x30
            Uca0SpiSendRec(BitConverter.GetBytes(Constants.wMSB_lower & (daten << 4))[0], true);
            return 0;
        }

        static byte Write2BytesToPga900(byte lsb, byte msb)
        {
            WriteByteToLSB(lsb);
            WriteByteToMSB(msb);
            return 0;
        }

        static byte ReadTxStatus()
        {
            byte data = 0x00;
            int i;
            for (i = 2; i > 0; i--)
            {
                Uca0SpiSendRec(Constants.rTxS_firstbyte, false);
                data = Uca0SpiSendRec(Constants.rTxS_secondbyte, false);
                Uca0SpiSendRec(Constants.rTxS_thirdbyte, true);
            }
            return data;
        }


        static byte SetTxStatus()
        {
            Uca0SpiSendRec(Constants.sTxS_firstbyte, false);
            Uca0SpiSendRec(Constants.sTxS_secondbyte, false);
            Uca0SpiSendRec(Constants.sTxS_thirdbyte, true);
            return 1;
        }

        static UInt16 Read2BytesFromPga900()
        {
            uint168_t data;
            data.uinta = 0x0000;
            UInt16 max_trys = 0;
            while (ReadTxStatus()==0 && (max_trys > 10)) 
		{ max_trys++; }

            for (int i = 2; i > 0; i--)
            {
                Uca0SpiSendRec(Constants.r2Bpga_firstbyte, false);
                data.uintb1 = Uca0SpiSendRec(Constants.r2Bpga_secondbyte, false);
                data.uintb0 = Uca0SpiSendRec(Constants.r2Bpga_thirdbyte, true);
            }

            SetTxStatus();
            return data.uinta;
        }

        static byte SendCMDToPGA(UInt16 CMD)
        {
            uint168_t cmd;
            cmd.uintb0 = 0x00;
            cmd.uintb1 = 0x01;
            cmd.uinta = CMD;

            Write2BytesToPga900(cmd.uintb0, cmd.uintb1);
            Console.WriteLine("Sent {0:X} {1:X} ", cmd.uintb1, cmd.uintb0);
            uint168_t response_co2;
            response_co2.uinta = Read2BytesFromPga900();
            response_co2.uintb0 = 0x00;
            response_co2.uintb1 = 0x01;
            Console.WriteLine("Response {0:X} {1:X}", response_co2.uintb1, response_co2.uintb0);
            if (response_co2.uinta == CMD)
            {
                return 1;
            }
            return 0;
        }

        static byte SwitchGaState()
        {
            SendCMDToPGA((UInt16)Constants.StateOfOperation_t.Gasanalysis);
            System.Threading.Thread.Sleep(50);
            SendCMDToPGA((UInt16)Constants.GasanalysisCommand_t.StartGasanalysis);
            System.Threading.Thread.Sleep(50);
            SendCMDToPGA((UInt16)Constants.TypeOfGasanalysis_t.ConstantOverTemperature);
            System.Threading.Thread.Sleep(50);
            SendCMDToPGA((UInt16)Constants.CO_TEMPERATURE);
            System.Threading.Thread.Sleep(50);
            SendCMDToPGA((UInt16)Constants.TypeOfGasanalysisParameter_t.DACValueParameter);
            System.Threading.Thread.Sleep(50);
            return 1;
        }

        static void GasMain()
        {
            SwitchGaState();
            //Sleep(1);
            //StartGa();
            //Sleep(1);
            //while (1)
            //{
            //    Sleep(10);
            //    printf("Data %d\n", ReadGa());
            //}
        }

        static void Main(string[] args)
        {


            // Check device
            UInt32 numOfDevices = 0;
            ftStatus = FT_CreateDeviceInfoList(ref numOfDevices);

            if (numOfDevices > 0)
            {
                byte[] sernum = new byte[16];
                byte[] desc = new byte[64];

                ftStatus = FT_GetDeviceInfoDetail(0, ref devInfo.Flags, ref devInfo.Type, ref devInfo.ID, ref devInfo.LocId,
                                            sernum, desc, ref devInfo.ftHandle);

                devInfo.SerialNumber = Encoding.ASCII.GetString(sernum, 0, 16);
                devInfo.Description = Encoding.ASCII.GetString(desc, 0, 64);
                devInfo.SerialNumber = devInfo.SerialNumber.Substring(0, devInfo.SerialNumber.IndexOf("\0"));
                Console.WriteLine(devInfo.SerialNumber);
                devInfo.Description = devInfo.Description.Substring(0, devInfo.Description.IndexOf("\0"));

                Console.WriteLine("Device Number: {0}", numOfDevices);
            }
            else
            {
                Console.WriteLine("No FTDI device");
                Console.WriteLine("NG! Press Enter to continue.");
                Console.ReadLine();
                return;
            }


            // Open device
            ftStatus = FT_OpenEx(devInfo.LocId, FT_OPEN_BY_LOCATION, ref ftHandle);

            if (ftStatus != FTDI.FT_STATUS.FT_OK)
            {
                Console.WriteLine("Open NG: {0}", ftStatus);
                Console.WriteLine("NG! Press Enter to continue.");
                Console.ReadLine();
                return;
            }


            // Set FT4222 clock
            FT4222_ClockRate ft4222_Clock = FT4222_ClockRate.SYS_CLK_60;

            ft42Status = FT4222_SetClock(ftHandle, FT4222_ClockRate.SYS_CLK_60);
            if (ft42Status != FT4222_STATUS.FT4222_OK)
            {
                Console.WriteLine("SetClock NG: {0}. Press Enter to continue.", ft42Status);
                Console.ReadLine();
                return;
            }
            else
            {
                Console.WriteLine("SetClock OK");

                ft42Status = FT4222_GetClock(ftHandle, ref ft4222_Clock);
                if (ft42Status != FT4222_STATUS.FT4222_OK)
                {
                    Console.WriteLine("GetClock NG: {0}. Press Enter to continue.", ft42Status);
                    Console.ReadLine();
                    return;
                }
                else
                {
                    Console.WriteLine("GetClock:" + ft4222_Clock);
                }
            }



            // Init FT4222 SPI Master
            ft42Status = FT4222_SPIMaster_Init(ftHandle, FT4222_SPIMode.SPI_IO_SINGLE, FT4222_SPIClock.CLK_DIV_128, FT4222_SPICPOL.CLK_IDLE_LOW, FT4222_SPICPHA.CLK_TRAILING, 0x01);
            if (ft42Status != FT4222_STATUS.FT4222_OK)
            {
                Console.WriteLine("Open NG: {0}", ft42Status);
                Console.WriteLine("NG! Press Enter to continue.");
                Console.ReadLine();
                return;
            }
            else
            {
                Console.WriteLine("Init FT4222 SPI Master OK");
            }

            ft42Status = FT4222_SPI_Reset(ftHandle);
            if (ft42Status != FT4222_STATUS.FT4222_OK)
            {
                Console.WriteLine("NG!");
            }
                // Read/Write data
                Console.WriteLine("Prepare to read/write data. Press Enter to continue.");
            Console.ReadLine();

            GasMain();

            Console.WriteLine("Done. Press Enter to continue.");
            Console.ReadLine();

            //End
            return;
        }
    }
}

/*
 *             ushort sizeTransferred = 0;
            byte[] readBuf = new byte[] { 0x00, 0x00, 0x00 };
            byte[] writeBuf = new byte[] { 0x01, 0x10, 0x30 };

            System.Threading.Thread.Sleep(1);

            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readBuf[0], ref writeBuf[0], 3, ref sizeTransferred, true);

            writeBuf = new byte[] { 0x01, 0x30, 0x00 };

            System.Threading.Thread.Sleep(1);

            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readBuf[0], ref writeBuf[0], 3, ref sizeTransferred, true);

            writeBuf = new byte[] { 0x00, 0x0C, 0x00 };

            System.Threading.Thread.Sleep(1);

            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readBuf[0], ref writeBuf[0], 3, ref sizeTransferred, true);

            System.Threading.Thread.Sleep(1);

            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readBuf[0], ref writeBuf[0], 3, ref sizeTransferred, true);

            writeBuf = new byte[] { 0x00, 0x80, 0x00 };

            System.Threading.Thread.Sleep(1);

            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readBuf[0], ref writeBuf[0], 3, ref sizeTransferred, true);

            System.Threading.Thread.Sleep(1);

            ft42Status = FT4222_SPIMaster_SingleReadWrite(ftHandle, ref readBuf[0], ref writeBuf[0], 3, ref sizeTransferred, true);

            if (ft42Status != FT4222_STATUS.FT4222_OK)
            {
                Console.WriteLine("Write NG: {0}", ft42Status);
            }
            else
            {
                // Show read/write data
                string strR = System.Text.Encoding.Default.GetString(readBuf);
                Console.WriteLine("R:["+ strR + "]");

                string strW = System.Text.Encoding.Default.GetString(writeBuf);
                Console.WriteLine("W:[" + strW + "]");

                Console.WriteLine("Read/Write OK, sizeTransferred: {0}", sizeTransferred);
            }

    */
