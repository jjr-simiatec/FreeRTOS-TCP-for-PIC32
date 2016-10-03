/*
 * FreeRTOS+FAT Library Test (mostly from FreeRTOS+TCP demo)
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

#include <inttypes.h>
#include <stdio.h>

#include <ff_ramdisk.h>

#include "TestHarness.h"

// The number and size of sectors that will make up the RAM disk
#define mainRAM_DISK_SECTOR_SIZE        512UL   // Currently fixed!
#define mainRAM_DISK_SECTORS            ((64UL * 1024UL) / mainRAM_DISK_SECTOR_SIZE)
#define mainIO_MANAGER_CACHE_SIZE       (4UL * mainRAM_DISK_SECTOR_SIZE)

#define mainRAM_DISK_NAME               "/ram"

static FF_Disk_t *pxRAMDisk;
static uint8_t ucRAMDisk[ mainRAM_DISK_SECTORS * mainRAM_DISK_SECTOR_SIZE ];

extern void vCreateAndVerifyExampleFiles( char *pcMountPath );
extern void vStdioWithCWDTest( const char *pcMountPath );
extern void vMultiTaskStdioWithCWDTest( const char *const pcMountPath, uint16_t usStackSizeWords );

void TestFATLibrary( void )
{
    ShowTestTitle("FAT LIBRARY TEST");
        
    printf("Going to create RAM disk @ %p\r\n", ucRAMDisk);
    
    /* Create the RAM disk. */
    pxRAMDisk = FF_RAMDiskInit(mainRAM_DISK_NAME, ucRAMDisk, mainRAM_DISK_SECTORS, mainIO_MANAGER_CACHE_SIZE);
    configASSERT( pxRAMDisk );
    
    /* Print out information on the RAM disk. */
    FF_RAMDiskShowPartition( pxRAMDisk );
    
    /* Create a few example files on the disk.  These are not deleted again. */
    vCreateAndVerifyExampleFiles( mainRAM_DISK_NAME );

    FF_Error_t nResult;
    uint32_t spaceLeft = FF_GetFreeSize(pxRAMDisk->pxIOManager, &nResult);
    
    printf("Remaining disk space: %" PRIo32 "\r\n", spaceLeft);
    
#if 0
    
    /* A few sanity checks only - can only be called after
    vCreateAndVerifyExampleFiles(). */
    #if( mainRUN_STDIO_TESTS_IN_MULTIPLE_TASK == 1 )
    {
        /* This continuously reads and writes to the disk - best not to use an
        flash disk. */
        vMultiTaskStdioWithCWDTest( mainRAM_DISK_NAME, configMINIMAL_STACK_SIZE * 5U );
    }
    #else
    {
        vStdioWithCWDTest( mainRAM_DISK_NAME );
    }
    #endif

#endif
}
