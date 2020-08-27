/*
 * FreeRTOS+TCP Server Test (mostly from FreeRTOS+TCP demo)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// RTOS
#include <FreeRTOS.h>
#include <task.h>
// TCP/IP Stack
#include <FreeRTOS_IP.h>
#include <FreeRTOS_TCP_Server.h>
// C Runtime
#include <stdbool.h>
#include <strings.h>

#include "ff_ramdiskex.h"
#include "TestHarness.h"
#include "TCPCommandConsole.h"

// The number and size of sectors that will make up the RAM disk
#define mainDISK_SECTOR_SIZE    512UL
#define mainRAM_DISK_SECTORS    ((20UL * 1024UL) / mainDISK_SECTOR_SIZE)

#if defined(__PIC32MX__)
static const char pBOARD_NAME[] = "pic32mx";
#elif defined(__PIC32MZ__)
static const char pBOARD_NAME[] = "pic32mz";
#endif

#define mainIO_MANAGER_CACHE_SIZE   (2UL * mainDISK_SECTOR_SIZE)

#define mainRAM_DISK_NAME           "/ram"
#define mainWEB_DISK_NAME           "/www"

#define RAND_MULTIPLIER     0x015A4E35UL
#define RAND_INCREMENT      1UL

#define TCP_CLI_TASK_STACK_SIZE     190U
#define TCP_CLI_SERVER_PORT         12345U

static const struct xSERVER_CONFIG xServerConfiguration[] =
{
    // Server type,  port number, backlog, root dir.
    {  eSERVER_HTTP, 80,          12,      configHTTP_ROOT },
#if (ipconfigUSE_FTP == 1)
    {  eSERVER_FTP,  21,          12,      ""              },
#endif
};

extern const uint8_t _binary_Web_Disk_img_start[];
extern const size_t _binary_Web_Disk_img_size;

static uint8_t ucRAMDisk[mainRAM_DISK_SECTORS * mainDISK_SECTOR_SIZE];

static FF_Disk_t *pxRAMDisk = NULL;
static FF_Disk_t *pxROMDisk = NULL;

static UBaseType_t s_ulNextRand;

ePingReplyStatus_t g_tPingReplyStatus;
uint16_t g_nPingReplySequence;

extern void vRegisterSampleCLICommands(void);
extern void vCreateAndVerifyExampleFiles(char *pcMountPath);

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
    if(eNetworkEvent == eNetworkUp)
    {
        xTaskNotifyGive(g_hTask2);
        xTaskNotifyGive(g_hPacketTask);
    }
}

void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
    g_tPingReplyStatus = eStatus;
    g_nPingReplySequence = usIdentifier;

    xTaskNotifyGive(g_hTask1);
}

BaseType_t xApplicationDNSQueryHook(const char *pcName)
{
    return strcasecmp(pcName, pBOARD_NAME) == 0;
}

portTASK_FUNCTION(Task2, pParams)
{
#if defined(PIC32_HAS_WORKING_TRNG)
    extern uint32_t TRNGRead32(void);
    s_ulNextRand = TRNGRead32();
#else
    time_t timeNow;
    time(&timeNow);

    s_ulNextRand = timeNow;
#endif

    // Create the RAM disk
    pxRAMDisk = FF_RAMDiskInit(mainRAM_DISK_NAME, ucRAMDisk, mainRAM_DISK_SECTORS, mainIO_MANAGER_CACHE_SIZE);
    configASSERT(pxRAMDisk);

    pxROMDisk = FF_DiskImageReadOnlyInit(mainWEB_DISK_NAME, _binary_Web_Disk_img_start, (size_t) &_binary_Web_Disk_img_size / mainDISK_SECTOR_SIZE, mainIO_MANAGER_CACHE_SIZE);
    configASSERT(pxROMDisk);

    vCreateAndVerifyExampleFiles(mainRAM_DISK_NAME);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TCPServer_t *pServers = FreeRTOS_CreateTCPServer(xServerConfiguration, _countof(xServerConfiguration));

    vRegisterSampleCLICommands();
    RegisterTestHarnessCLICommands();

    vStartTCPCommandInterpreterTask(TCP_CLI_TASK_STACK_SIZE, TCP_CLI_SERVER_PORT, tskIDLE_PRIORITY + 1);

    for( ; ; )
    {
        FreeRTOS_TCPServerWork(pServers, pdMS_TO_TICKS(200));
    }
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress, uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress, uint16_t usDestinationPort )
{
	s_ulNextRand = (RAND_MULTIPLIER * s_ulNextRand) + RAND_INCREMENT;
	return (int)(s_ulNextRand >> 16U) & 0x7FFFUL;
}

