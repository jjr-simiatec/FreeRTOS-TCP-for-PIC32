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

#include <FreeRTOS.h>
#include <task.h>

#include <FreeRTOS_IP.h>
#include <FreeRTOS_TCP_Server.h>
#include <ff_ramdisk.h>

#include "TestHarness.h"

// The number and size of sectors that will make up the RAM disk
#define mainRAM_DISK_SECTOR_SIZE        512UL   // Currently fixed!
#define mainRAM_DISK_SECTORS            ((64UL * 1024UL) / mainRAM_DISK_SECTOR_SIZE)
#define mainIO_MANAGER_CACHE_SIZE       (4UL * mainRAM_DISK_SECTOR_SIZE)

#define mainRAM_DISK_NAME               "/ram"

static const struct xSERVER_CONFIG xServerConfiguration[] =
{
    /* Server type,     port number,    backlog,    root dir. */
    { eSERVER_HTTP,     80,             12,         configHTTP_ROOT },

#if( mainCREATE_FTP_SERVER == 1 )
    { eSERVER_FTP,      21,             12,         "" }
#endif /* mainCREATE_FTP_SERVER */
};

#if defined(__PIC32MZ__)
static uint8_t ucRAMDisk[mainRAM_DISK_SECTORS * mainRAM_DISK_SECTOR_SIZE];
#endif

static FF_Disk_t *pxRAMDisk = NULL;
extern TaskHandle_t g_hTask2;

extern void vCreateAndVerifyExampleFiles(char *pcMountPath);

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
    if(eNetworkEvent == eNetworkUp)
    {
        xTaskNotifyGive(g_hTask2);
    }
}

void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
#ifdef __DEBUG
    __builtin_software_breakpoint();
#endif // __DEBUG
}

BaseType_t xApplicationDNSQueryHook(const char *pcName)
{
#ifdef __DEBUG
//    __builtin_software_breakpoint();
#endif // __DEBUG
    
    return 0;
}

#if defined(__PIC32MX__)

portTASK_FUNCTION(Task2, pParams)
{
    for( ; ; )
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
        LATDINV = 0x01;
    }
}

#else

portTASK_FUNCTION(Task2, pParams)
{
    // Create the RAM disk
    pxRAMDisk = FF_RAMDiskInit(mainRAM_DISK_NAME, ucRAMDisk, mainRAM_DISK_SECTORS, mainIO_MANAGER_CACHE_SIZE);
    configASSERT(pxRAMDisk);

    vCreateAndVerifyExampleFiles(mainRAM_DISK_NAME);
    
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    TCPServer_t *pServers = FreeRTOS_CreateTCPServer(xServerConfiguration, _countof(xServerConfiguration));
            
    for( ; ; )
    {
        FreeRTOS_TCPServerWork(pServers, pdMS_TO_TICKS(200));
    }
}

#endif