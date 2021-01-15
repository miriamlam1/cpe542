/* lab1: DMA
* Jae Park and Miriam Lam
* 1/15/2020
* 
* INPUTS: a pointer to a memory address containing the data you'd like to move
* OUTPUTS: a pointer to a memory address where you'd like the data to be placed
* 
* Code from: https://iosoft.blog/2020/05/25/raspberry-pi-dma-programming/
* https://github.com/jbentham/rpi/blob/master/rpi_dma_test.c
* with slight modifications to meet spec
*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

// DMA register definitions
void *virt_dma_regs, *bus_dma_mem;
#define DMA_CHAN        5
#define DMA_PWM_DREQ    5
#define DMA_BASE        (PHYS_REG_BASE + 0x007000)
#define DMA_CS          (DMA_CHAN*0x100)
#define DMA_CONBLK_AD   (DMA_CHAN*0x100 + 0x04)
#define DMA_TI          (DMA_CHAN*0x100 + 0x08)
#define DMA_SRCE_AD     (DMA_CHAN*0x100 + 0x0c)
#define DMA_DEST_AD     (DMA_CHAN*0x100 + 0x10)
#define DMA_TXFR_LEN    (DMA_CHAN*0x100 + 0x14)
#define DMA_STRIDE      (DMA_CHAN*0x100 + 0x18)
#define DMA_NEXTCONBK   (DMA_CHAN*0x100 + 0x1c)
#define DMA_DEBUG       (DMA_CHAN*0x100 + 0x20)
#define DMA_ENABLE      0xff0
#define VIRT_DMA_REG(a) ((volatile uint32_t *)((uint32_t)virt_dma_regs + a))
char *dma_regstrs[] = {"DMA CS", "CB_AD", "TI", "SRCE_AD", "DEST_AD",
    "TFR_LEN", "STRIDE", "NEXT_CB", "DEBUG", ""};

// DMA control block (must be 32-byte aligned)
typedef struct {
    uint32_t ti,    // Transfer info
        srce_ad,    // Source address
        dest_ad,    // Destination address
        tfr_len,    // Transfer length
        stride,     // Transfer stride
        next_cb,    // Next control block
        debug,      // Debug register
        unused;
} DMA_CB __attribute__ ((aligned(32)));
#define DMA_CB_DEST_INC (1<<4)
#define DMA_CB_SRC_INC  (1<<8)

// Virtual memory for DMA descriptors and data buffers (uncached)
void *virt_dma_mem;

// Convert virtual DMA data address to a bus address
#define BUS_DMA_MEM(a)  ((uint32_t)a-(uint32_t)virt_dma_mem+(uint32_t)bus_dma_mem)

// Start DMA, given first control block
void start_dma(DMA_CB *cbp)
{
    *VIRT_DMA_REG(DMA_CONBLK_AD) = BUS_DMA_MEM(cbp);
    *VIRT_DMA_REG(DMA_CS) = 2;       // Clear 'end' flag
    *VIRT_DMA_REG(DMA_DEBUG) = 7;    // Clear error bits
    *VIRT_DMA_REG(DMA_CS) = 1;       // Start DMA
}

// DMA memory-to-memory test
int dma_test_mem_transfer(void)
{
    DMA_CB *cbp = (DMA_CB*)virt_dma_mem;
    char *srce = (char *)(cbp+1);
    char *dest = srce + 0x100;
 
    strcpy(srce, "memory transfer OK");
    memset(cbp, 0, sizeof(DMA_CB));
    cbp->ti = DMA_CB_SRC_INC | DMA_CB_DEST_INC;
    cbp->srce_ad = BUS_DMA_MEM(srce);
    cbp->dest_ad = BUS_DMA_MEM(dest);
    cbp->tfr_len = strlen(srce) + 1;
    start_dma(cbp);
    usleep(10);
#if DEBUG
    disp_dma();
#endif
    printf("DMA test: %s\n", dest[0] ? dest : "failed");
    return(dest[0] != 0);
}

int main(int argc, char* arv[]){
    dma_test_mem_transfer();
}