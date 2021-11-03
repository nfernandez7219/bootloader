/*******************************************************************************
  UART Bootloader Source File

  File Name:
    bootloader.c

  Summary:
    This file contains source code necessary to execute UART bootloader.

  Description:
    This file contains source code necessary to execute UART bootloader.
    It implements bootloader protocol which uses UART peripheral to download
    application firmware into internal flash from HOST-PC.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "definitions.h"
#include <device.h>

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

#define FLASH_START             (0UL)
#define FLASH_LENGTH            (1048576UL)
#define PAGE_SIZE               (512UL)
#define ERASE_BLOCK_SIZE        (8192UL)
#define PAGES_IN_ERASE_BLOCK    (ERASE_BLOCK_SIZE / PAGE_SIZE)

#define BOOTLOADER_SIZE         8192

#define APP_START_ADDRESS       (0x2000UL)

#define GUARD_OFFSET            0
#define CMD_OFFSET              2
#define ADDR_OFFSET             0
#define SIZE_OFFSET             1
#define DATA_OFFSET             1
#define CRC_OFFSET              0

#define CMD_SIZE                1
#define GUARD_SIZE              4
#define SIZE_SIZE               4
#define OFFSET_SIZE             4
#define CRC_SIZE                4
#define HEADER_SIZE             (GUARD_SIZE + SIZE_SIZE + CMD_SIZE)
#define DATA_SIZE               ERASE_BLOCK_SIZE

#define WORDS(x)                ((int)((x) / sizeof(uint32_t)))

#define OFFSET_ALIGN_MASK       (~ERASE_BLOCK_SIZE + 1)
#define SIZE_ALIGN_MASK         (~PAGE_SIZE + 1)

#define BTL_GUARD               (0x5048434DUL)

#define SIGNATURE1              (0xAA55FADE)
#define SIGNATURE2              (0x55AAC0DE)

enum
{
    BL_CMD_UNLOCK       = 0xa0,
    BL_CMD_DATA         = 0xa1,
    BL_CMD_VERIFY       = 0xa2,
    BL_CMD_RESET        = 0xa3,
    BL_CMD_BKSWAP_RESET = 0xa4,
};

enum
{
    BL_RESP_OK          = 0x50,
    BL_RESP_ERROR       = 0x51,
    BL_RESP_INVALID     = 0x52,
    BL_RESP_CRC_OK      = 0x53,
    BL_RESP_CRC_FAIL    = 0x54,
};

struct binary_header {
        uint32_t sig1;
        uint32_t sig2;
        uint32_t bin_size;
        uint32_t crc32;
};

// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

static uint32_t input_buffer[WORDS(OFFSET_SIZE + DATA_SIZE)];

static uint32_t flash_data[WORDS(DATA_SIZE)];
static uint32_t flash_addr          = 0;

static uint32_t unlock_begin        = 0;
static uint32_t unlock_end          = 0;

static uint8_t  input_command       = 0;

static bool     packet_received     = false;
static bool     flash_data_ready    = false;

// *****************************************************************************
// *****************************************************************************
// Section: Bootloader Local Functions
// *****************************************************************************
// *****************************************************************************

/* Function to Generate CRC using the device service unit peripheral on programmed data */
static uint32_t crc_generate(void)
{
    uint32_t addr = unlock_begin;
    uint32_t size = unlock_end - unlock_begin;
    uint32_t crc  = 0;

    PAC_PeripheralProtectSetup (PAC_PERIPHERAL_DSU, PAC_PROTECTION_CLEAR);

    DSU_CRCCalculate (addr, size, 0xffffffff, &crc);

    PAC_PeripheralProtectSetup (PAC_PERIPHERAL_DSU, PAC_PROTECTION_SET);

    return crc;
}

/* Function to receive application firmware via UART/USART */
static void input_task(void)
{
    static uint32_t ptr             = 0;
    static uint32_t size            = 0;
    static bool     header_received = false;
    uint8_t         *byte_buf       = (uint8_t *)&input_buffer[0];
    uint8_t         input_data      = 0;

    if (packet_received == true)
    {
        return;
    }

    if (SERCOM0_USART_ReceiverIsReady() == false)
    {
        return;
    }

    input_data = SERCOM0_USART_ReadByte();

    /* Check if 100 ms have elapsed */
    if (SYSTICK_TimerPeriodHasExpired())
    {
        header_received = false;
    }

    if (header_received == false)
    {
        byte_buf[ptr++] = input_data;

        if (ptr == HEADER_SIZE)
        {
            if (input_buffer[GUARD_OFFSET] != BTL_GUARD)
            {
                SERCOM0_USART_WriteByte(BL_RESP_ERROR);
            }
            else
            {
                size            = input_buffer[SIZE_OFFSET];
                input_command   = (uint8_t)input_buffer[CMD_OFFSET];
                header_received = true;
            }

            ptr = 0;
        }
    }
    else if (header_received == true)
    {
        if (ptr < size)
        {
            byte_buf[ptr++] = input_data;
        }

        if (ptr == size)
        {
            ptr = 0;
            size = 0;
            packet_received = true;
            header_received = false;
        }
    }

    SYSTICK_TimerRestart();
}

/* Function to process the received command */
static void command_task(void)
{
    uint32_t i;

    if (BL_CMD_UNLOCK == input_command)
    {
        uint32_t begin  = (input_buffer[ADDR_OFFSET] & OFFSET_ALIGN_MASK);

        uint32_t end    = begin + (input_buffer[SIZE_OFFSET] & SIZE_ALIGN_MASK);

        if (end > begin && end <= (FLASH_START + FLASH_LENGTH))
        {
            unlock_begin = begin;
            unlock_end = end;
            SERCOM0_USART_WriteByte(BL_RESP_OK);
        }
        else
        {
            unlock_begin = 0;
            unlock_end = 0;
            SERCOM0_USART_WriteByte(BL_RESP_ERROR);
        }
    }
    else if (BL_CMD_DATA == input_command)
    {
        flash_addr = (input_buffer[ADDR_OFFSET] & OFFSET_ALIGN_MASK);

        if (unlock_begin <= flash_addr && flash_addr < unlock_end)
        {
            for (i = 0; i < WORDS(DATA_SIZE); i++)
                flash_data[i] = input_buffer[i + DATA_OFFSET];

            flash_data_ready = true;

            SERCOM0_USART_WriteByte(BL_RESP_OK);
        }
        else
        {
            SERCOM0_USART_WriteByte(BL_RESP_ERROR);
        }
    }
    else if (BL_CMD_VERIFY == input_command)
    {
        uint32_t crc        = input_buffer[CRC_OFFSET];
        uint32_t crc_gen    = 0;

        crc_gen = crc_generate();

        if (crc == crc_gen)
            SERCOM0_USART_WriteByte(BL_RESP_CRC_OK);
        else
            SERCOM0_USART_WriteByte(BL_RESP_CRC_FAIL);
    }
    else if (BL_CMD_BKSWAP_RESET == input_command)
    {
        SERCOM0_USART_WriteByte(BL_RESP_OK);

        while(SERCOM0_USART_TransmitComplete() == false);

        NVMCTRL_BankSwap();
    }
    else if (BL_CMD_RESET == input_command)
    {
        SERCOM0_USART_WriteByte(BL_RESP_OK);

        while(SERCOM0_USART_TransmitComplete() == false);

        NVIC_SystemReset();
    }
    else
    {
        SERCOM0_USART_WriteByte(BL_RESP_INVALID);
    }

    packet_received = false;
}

/* Function to program received application firmware data into internal flash */
static void flash_task(void)
{
    uint32_t addr       = flash_addr;
    uint32_t page       = 0;
    uint32_t write_idx  = 0;

    // Lock region size is always bigger than the row size
    NVMCTRL_RegionUnlock(addr);

    while(NVMCTRL_IsBusy() == true)
        input_task();

    /* Erase the Current sector */
    NVMCTRL_BlockErase(addr);

    /* Receive Next Bytes while waiting for erase to complete */
    while(NVMCTRL_IsBusy() == true)
        input_task();

    for (page = 0; page < PAGES_IN_ERASE_BLOCK; page++)
    {
        NVMCTRL_PageWrite(&flash_data[write_idx], addr);

        while(NVMCTRL_IsBusy() == true)
            input_task();

        addr += PAGE_SIZE;
        write_idx += WORDS(PAGE_SIZE);
    }

    flash_data_ready = false;
}

unsigned long crc32(unsigned long inCrc32, const void *buf, size_t bufLen) {
        static const unsigned long crcTable[256] = {
         0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,0xE963A535,
         0x9E6495A3,0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,0x09B64C2B,0x7EB17CBD,
         0xE7B82D07,0x90BF1D91,0x1DB71064,0x6AB020F2,0xF3B97148,0x84BE41DE,0x1ADAD47D,
         0x6DDDE4EB,0xF4D4B551,0x83D385C7,0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,
         0x14015C4F,0x63066CD9,0xFA0F3D63,0x8D080DF5,0x3B6E20C8,0x4C69105E,0xD56041E4,
         0xA2677172,0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,0x35B5A8FA,0x42B2986C,
         0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,0x26D930AC,
         0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,0xCFBA9599,0xB8BDA50F,
         0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,0x2F6F7C87,0x58684C11,0xC1611DAB,
         0xB6662D3D,0x76DC4190,0x01DB7106,0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,
         0x9FBFE4A5,0xE8B8D433,0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,
         0x086D3D2D,0x91646C97,0xE6635C01,0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,
         0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,0x65B0D9C6,0x12B7E950,0x8BBEB8EA,
         0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,0x4DB26158,0x3AB551CE,
         0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,0xA4D1C46D,0xD3D6F4FB,0x4369E96A,
         0x346ED9FC,0xAD678846,0xDA60B8D0,0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,
         0x5005713C,0x270241AA,0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,
         0xCE61E49F,0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,
         0xB7BD5C3B,0xC0BA6CAD,0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,0xEAD54739,
         0x9DD277AF,0x04DB2615,0x73DC1683,0xE3630B12,0x94643B84,0x0D6D6A3E,0x7A6A5AA8,
         0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,0xF00F9344,0x8708A3D2,0x1E01F268,
         0x6906C2FE,0xF762575D,0x806567CB,0x196C3671,0x6E6B06E7,0xFED41B76,0x89D32BE0,
         0x10DA7A5A,0x67DD4ACC,0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,0xD6D6A3E8,
         0xA1D1937E,0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
         0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,0x316E8EEF,
         0x4669BE79,0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,0xCC0C7795,0xBB0B4703,
         0x220216B9,0x5505262F,0xC5BA3BBE,0xB2BD0B28,0x2BB45A92,0x5CB36A04,0xC2D7FFA7,
         0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,
         0x9C0906A9,0xEB0E363F,0x72076785,0x05005713,0x95BF4A82,0xE2B87A14,0x7BB12BAE,
         0x0CB61B38,0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,0x86D3D2D4,0xF1D4E242,
         0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,0x88085AE6,
         0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,0x616BFFD3,0x166CCF45,
         0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,0xA7672661,0xD06016F7,0x4969474D,
         0x3E6E77DB,0xAED16A4A,0xD9D65ADC,0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,
         0x47B2CF7F,0x30B5FFE9,0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,
         0xCDD70693,0x54DE5729,0x23D967BF,0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,
         0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D
        };
        unsigned long crc32;
        unsigned char *byteBuf;
        size_t i;

        /** accumulate crc32 for buffer **/
        crc32 = inCrc32 ^ 0xFFFFFFFF;
        byteBuf = (unsigned char *)buf;
        for (i = 0; i < bufLen; i++) {
                crc32 = (crc32 >> 8) ^ crcTable[(crc32 ^ byteBuf[i]) & 0xFF];
        }
        return crc32 ^ 0xFFFFFFFF;
}

/* binary header must be located somewhere within the first 8k of application
 * firmware */
struct binary_header *find_binary_header(void)
{
    uint32_t *start = (uint32_t *)(APP_START_ADDRESS);
    uint32_t *end = start + (ERASE_BLOCK_SIZE/sizeof(uint32_t));
    struct binary_header *hdr = NULL;
#if 0
    static const char print_report[] = "finding binary header\r\n";
    static const char print_found[] = "found!\r\n";
    static const char print_not_found[] = "not found!\r\n";
    static const char size_is[] = "size is: ";
    static const char checksum_is[] = "checksum is: ";
    SERCOM0_USART_Write((char *)print_report, sizeof(print_report));
#endif
    
    for ( ; (start-1) < end; start++) {
        if (start[0] == SIGNATURE1 && start[1] == SIGNATURE2) {
            hdr = (struct binary_header *)start;
#if 0
            SERCOM0_USART_Write((char *)print_found, sizeof(print_found));

            /* report size */
            SERCOM0_USART_Write((char *)size_is, sizeof(size_is));
            SERCOM0_USART_WriteByte((int)(hdr->bin_size >> 0) & 0xFF);
            SERCOM0_USART_WriteByte((int)(hdr->bin_size >> 8) & 0xFF);
            SERCOM0_USART_WriteByte((int)(hdr->bin_size >> 16) & 0xFF);
            SERCOM0_USART_WriteByte((int)(hdr->bin_size >> 24) & 0xFF);
            SERCOM0_USART_Write("\r\n", 2);

            /* report checksum */
            SERCOM0_USART_Write((char *)checksum_is, sizeof(checksum_is));
            SERCOM0_USART_WriteByte((int)(hdr->crc32 >> 0) & 0xFF);
            SERCOM0_USART_WriteByte((int)(hdr->crc32 >> 8) & 0xFF);
            SERCOM0_USART_WriteByte((int)(hdr->crc32 >> 16) & 0xFF);
            SERCOM0_USART_WriteByte((int)(hdr->crc32 >> 24) & 0xFF);
            SERCOM0_USART_Write("\r\n", 2);
#endif            
            return hdr;
            
            break;
        }
    }
    //SERCOM0_USART_Write((char *)print_not_found, sizeof(print_not_found));
    return hdr;
}

// *****************************************************************************
// *****************************************************************************
// Section: Bootloader Global Functions
// *****************************************************************************
// *****************************************************************************

void run_Application(void)
{
    uint32_t msp            = *(uint32_t *)(APP_START_ADDRESS);
    uint32_t reset_vector   = *(uint32_t *)(APP_START_ADDRESS + 4);

    uint8_t *start;
    uint8_t *end;
    struct binary_header *hdr;
    uint8_t *tmp;
    uint32_t checksum = 0;
    uint16_t nvm_status;

    if (msp == 0xffffffff)
    {
        return;
    }

    /* there is firmware, but header signature was not found... this might
     * mean the signature was corrupted, so we treat the entire firmware also
     * as corrupted. boot into bootloader mode instead of loading the firmware.
     */
    if (!(hdr = find_binary_header())) {
        return;
    }
    start = (uint8_t *)(APP_START_ADDRESS);
    end = start + hdr->bin_size;
    tmp = (uint8_t *)hdr;
    
    /* compute the initial checksum, skip the header and continue computing the 
     * checksum until we are done with the entire firmware. */
    checksum = crc32(checksum, start, (size_t)(tmp - start));
    tmp = tmp + sizeof(struct binary_header);
    checksum = crc32(checksum, tmp, (size_t)(end - tmp));
   
#if 0
    static char const checksum_computed[] = "computed checksum is: ";
    SERCOM0_USART_Write((char *)checksum_computed, sizeof(checksum_computed));
    SERCOM0_USART_WriteByte((checksum >> 0) & 0xFF);
    SERCOM0_USART_WriteByte((checksum >> 8) & 0xFF);
    SERCOM0_USART_WriteByte((checksum >> 16) & 0xFF);
    SERCOM0_USART_WriteByte((checksum >> 24) & 0xFF);
    
    static char const checksum_matched[] = "checksums matched! booting firmware...\r\n";
    static char const checksum_not_matched[] = "checksums did match...\r\n";
    
    if (hdr->crc32 == checksum) {
        SERCOM0_USART_Write((char *)checksum_matched, sizeof(checksum_matched));
    } else {
        SERCOM0_USART_Write((char *)checksum_not_matched, sizeof(checksum_not_matched));
    }
#endif

    /* now we compare if checksums match. if they do, continue with the 
     * rest of normal bootup process. */
    if (checksum != hdr->crc32) {
        /* if they don't match, then we see if we can bootup failsafe firmware. 
         * if we are booting from copy A (BankA), then switch to copy B (BankB)
         */
        nvm_status  = NVMCTRL_StatusGet();
        if ((nvm_status & (NVMCTRL_STATUS_AFIRST_Msk)) != 0) {
            NVMCTRL_BankSwap();
        } else {
            /* if we are booting from copy B (BankB) and we still encounter 
             * checksum mismatch, then we encountered a gross error in our setup.
             * both copy A and the failsafe copy B are corrupted. we go back to
             * bootloader mode.
             * 
             * maybe we can also light up a red LED to signal this gross error 
             * here??
             */
            // led_assert();
            return;
        }
    }
    
    __set_MSP(msp);
    asm("bx %0"::"r" (reset_vector));
}

/* bootloading algorithm:
 * 
 * the bootloader first checks if there is an actual firmware to load in offset
 * 0x2000. it does this mainly by checking if the block at 0x2000 is erased or
 * not. if it is erased, then there is really no reason to load anything and 
 * bootloader will simply just continue booting itself. 
 *
 * if it is not erased, then there is firmware to load. but bootloader now has
 * to check if firmware instructed it to go into bootloader mode. it does this
 * by checking for a predefined pattern in the RAM space.
 * 
 * firmware triggers a bootup to bootloader mode by writing a predefined pattern
 * to the RAM space and issuing a soft reset.
 */
bool __WEAK bootloader_Trigger(void)
{
    uint32_t *entry_point = (uint32_t *)APP_START_ADDRESS;
    static const uint32_t *sram = (uint32_t *)BTL_TRIGGER_RAM_START;

    /* if there is nothing to load... continue booting up bootloader */
    if (entry_point[0] == 0xFFFFFFFF) {
        return true;
    }

    /* if we reach here, there is firmware to load... so check if firmware
     * instructed us to go into bootloader mode or not. */
    if ((sram[0] == TRIGGER_SIGNATURE0) &&
        (sram[1] == TRIGGER_SIGNATURE1)) {
            return true;
    }

    return false;
}

void bootloader_Tasks(void)
{
    while (1)
    {
        input_task();

        if (flash_data_ready)
            flash_task();
        else if (packet_received)
            command_task();
    }
}
