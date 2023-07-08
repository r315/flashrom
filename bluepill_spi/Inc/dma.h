#ifndef _dma_h_
#define _dma_h_

#include <stddef.h>
#include <stdint.h>

typedef struct dmactrl{
   void *per;
   void *stream;
   void *src;
   void *dst;
   uint32_t len;
   struct {
      uint32_t ssize : 5;
      uint32_t dsize : 5;
      uint32_t single : 1;
      uint32_t dir : 2;
   };
   void (*eot)(void);
}dmatype_t;

enum {
   DMA_DIR_P2P,
   DMA_DIR_P2M,
   DMA_DIR_M2P,
   DMA_DIR_M2M
};

/**
 * @brief Request dma controller/channel
 * \param ctrl    : pointer to hold controller and channel
 * \param req     : Request peripheral
 * */
void DMA_Config(dmatype_t *dma, uint32_t req);

void DMA_Init(dmatype_t *dma);

void DMA_Start(dmatype_t *dma);

void DMA_Stop(dmatype_t *dma);

#endif /* _dma_h_ */