//
// pi/controller.c: Parallel interface controller.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "bus/address.h"
#include "bus/controller.h"
#include "dd/controller.h"
#include "pi/controller.h"
#include "pi/is_viewer.h"
#include "ri/controller.h"
#include "vr4300/interface.h"
#include <assert.h>

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *pi_register_mnemonics[NUM_PI_REGISTERS] = {
#define X(reg) #reg,
#include "pi/registers.md"
#undef X
};
#endif

static int pi_dma_read(struct pi_controller *pi);
static int pi_dma_write(struct pi_controller *pi);

// Advances the controller by one clock cycle.
void pi_cycle_(struct pi_controller *pi) {

  // DMA engine is finishing up with one entry.
  if (pi->bytes_to_copy > 0) {
    uint32_t bytes = pi->bytes_to_copy;

    // XXX: Defer actual movement of bytes until... now.
    // This is a giant hack; bytes should be DMA'd slowly.
    pi->is_dma_read ? pi_dma_read(pi) : pi_dma_write(pi);

    pi->regs[PI_DRAM_ADDR_REG] += bytes;
    pi->regs[PI_CART_ADDR_REG] += bytes;
    pi->regs[PI_STATUS_REG] &= ~PI_STATUS_DMA_BUSY;
    pi->regs[PI_STATUS_REG] |= PI_STATUS_INTERRUPT;

    signal_rcp_interrupt(pi->bus->vr4300, MI_INTR_PI);

    pi->bytes_to_copy = 0;
    return;
  }
}

// Copies data from RDRAM to the PI
static int pi_dma_read(struct pi_controller *pi) {
  uint32_t dest = pi->regs[PI_CART_ADDR_REG] & 0xFFFFFFE;
  uint32_t source = pi->regs[PI_DRAM_ADDR_REG] & 0x7FFFFF;
  uint32_t length = (pi->regs[PI_RD_LEN_REG] & 0xFFFFFF) + 1;

  if (length & 7)
    length = (length + 7) & ~7;

  // SRAM and FlashRAM
  if (dest >= 0x08000000 && dest < 0x08010000) {
    uint32_t addr = dest & 0x00FFFFF;

    // SRAM
    if (pi->sram->ptr != NULL && addr + length <= 0x8000)
      memcpy((uint8_t *) (pi->sram->ptr) + addr, pi->bus->ri->ram + source, length);

    // FlashRAM: Save the RDRAM destination address. Writing happens
    // after the system sends the flash write command (handled in
    // write_flashram)
    else if (pi->flashram.data != NULL && pi->flashram.mode == FLASHRAM_WRITE)
      pi->flashram.rdram_pointer = source;
  }

  else if ((source & 0x05000000) == 0x05000000)
    dd_dma_read(pi->bus->dd, source, dest, length);

  return 0;
}

// Copies data from the the PI into RDRAM.
static int pi_dma_write(struct pi_controller *pi) {
  uint32_t dest = pi->regs[PI_DRAM_ADDR_REG] & 0x7FFFFF;
  uint32_t source = pi->regs[PI_CART_ADDR_REG] & 0xFFFFFFE;
  uint32_t length = (pi->regs[PI_WR_LEN_REG] & 0xFFFFFF) + 1;

  if (length & 7)
    length = (length + 7) & ~7;

  if (pi->bus->dd->ipl_rom && (source & 0x06000000) == 0x06000000) {
    source &= 0x003FFFFF;

    if (source + length > 0x003FFFFF)
      length = 0x003FFFFF - source;

    memcpy(pi->bus->ri->ram + dest, pi->bus->dd->ipl_rom + source, length);
  }

  else if ((source & 0x05000000) == 0x05000000)
    dd_dma_write(pi->bus->dd, source, dest, length);

  // SRAM and FlashRAM
  else if (source >= 0x08000000 && source < 0x08010000) {
    uint32_t addr = source & 0x00FFFFF;

    if (pi->sram->ptr != NULL && addr + length <= 0x8000)
      memcpy(pi->bus->ri->ram + dest, (const uint8_t *) (pi->sram->ptr) + addr, length);

    else if (pi->flashram.data != NULL) {
      // SRAM
      if (pi->flashram.mode == FLASHRAM_STATUS) {
        uint64_t status = htonll(pi->flashram.status);
        memcpy(pi->bus->ri->ram + dest, &status, 8);
      }

      // FlashRAM
      else if (pi->flashram.mode == FLASHRAM_READ)
        memcpy(pi->bus->ri->ram + dest, pi->flashram.data + addr * 2, length);
    }
  }

  else if (source >= 0x18000000 && source < 0x18400000) {
    // TODO: 64DD modem
  }

  else if (pi->rom) {
    if (source + length > pi->rom_size) {
      unsigned i;

      // TODO: Check for correctness against hardware.
      // Is this the right address to use for open bus?
      for (i = (pi->regs[PI_CART_ADDR_REG] + pi->rom_size + 3) & ~0x3;
          i < pi->regs[PI_CART_ADDR_REG] + length; i += 4) {
        uint32_t word = (i >> 16) | (i & 0xFFFF0000);
        memcpy(pi->bus->ri->ram + dest, &word, sizeof(word));
      }

      length = pi->rom_size - source;
    }

    // TODO: Very hacky.
    if (source < pi->rom_size)
      memcpy(pi->bus->ri->ram + dest, pi->rom + source, length);
  }

  return 0;
}

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <io.h>
#else
#include <sys/socket.h>
#include <sys/un.h>
#endif

#include <unistd.h>

#define PORTNUM 2159

static int serial_socket;
static int serial_client;

// Initializes the PI.
int pi_init(struct pi_controller *pi, struct bus_controller *bus,
  const uint8_t *rom, size_t rom_size, const struct save_file *sram,
  const struct save_file *flashram, struct is_viewer *is_viewer) {
  pi->bus = bus;
  pi->rom = rom;
  pi->rom_size = rom_size;
  pi->sram = sram;
  pi->flashram_file = flashram;
  pi->flashram.data = flashram->ptr;
  pi->is_viewer = is_viewer;

  pi->bytes_to_copy = 0;

#ifdef _WIN32
WSADATA wsa;
WSAStartup(MAKEWORD(2,2), &wsa);
#endif

  // Code to create unix socket for serial
  int return_code = 0;
  struct sockaddr_in addr = {0};
  if ((serial_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    printf("Failed to create serial unix socket\n");
    return_code = 1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  //addr.sin_addr.s_addr = IPADDR_ANY;
  addr.sin_port = htons(PORTNUM);


  if(bind(serial_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    printf("Failed to bind socket\n");
    return_code = 1;
  }

  if(listen(serial_socket, 1) < 0) {
    printf("Failed to listen on socket\n");
    return_code = 1;
  }

  if((serial_client = accept(serial_socket, NULL, NULL)) < 0) {
    printf("Failed to accept client\n");
    return_code = 1;
  }

  return return_code;
}

// Reads a word from cartridge ROM.
int read_cart_rom(void *opaque, uint32_t address, uint32_t *word) {
  struct pi_controller *pi = (struct pi_controller *) opaque;
  unsigned offset = (address - ROM_CART_BASE_ADDRESS) & ~0x3;
  char data;

  if (pi->is_viewer && is_viewer_map(pi->is_viewer, address)){
    printf("In is_viewer\n");
    return read_is_viewer(pi->is_viewer, address, word);
  }

  if(address == 0x18000000) {
    if((recv(serial_client, &data, sizeof(data), 0)) == -1) {
      printf("Failed to read data\n");
      return 1;
    }

    *word = data;
    //printf("Reading %c\n", (char)data);
    return 0;
  }

  // TODO: Need to figure out correct behaviour.
  // Should this even happen to begin with?
  if (pi->rom == NULL || offset > (pi->rom_size - sizeof(*word))) {
    *word = (address >> 16) | (address & 0xFFFF0000);
    return 0;
  }

  memcpy(word, pi->rom + offset, sizeof(*word));
  *word = byteswap_32(*word);
  return 0;
}

// Reads a word from the PI MMIO register space.
int read_pi_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct pi_controller *pi = (struct pi_controller *) opaque;
  unsigned offset = address - PI_REGS_BASE_ADDRESS;
  enum pi_register reg = (offset >> 2);

  *word = pi->regs[reg];

  debug_mmio_read(pi, pi_register_mnemonics[reg], *word);
  return 0;
}

// Writes a word to cartridge ROM.
int write_cart_rom(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct pi_controller *pi = (struct pi_controller *) opaque;
  char data = word;
  if(address == 0x18000000) {
    //printf("Writing %c\n", (char)word);
    if((send(serial_client, &data, sizeof(data), 0)) == -1) {
      printf("Failed to write data\n");
      return 1;
    }
  }

  if(address == 0x18000004) {
    printf("Wrote value 0x%x\n", word);
  }

  if (pi->is_viewer && is_viewer_map(pi->is_viewer, address))
    return write_is_viewer(pi->is_viewer, address, word, dqm);

  return 0;
}

// Writes a word to the PI MMIO register space.
int write_pi_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct pi_controller *pi = (struct pi_controller *) opaque;
  unsigned offset = address - PI_REGS_BASE_ADDRESS;
  enum pi_register reg = (offset >> 2);

  debug_mmio_write(pi, pi_register_mnemonics[reg], word, dqm);

  if (reg == PI_STATUS_REG) {
    if (word & PI_STATUS_RESET_CONTROLLER)
      pi->regs[reg] &= ~(PI_STATUS_IS_BUSY | PI_STATUS_ERROR);

    if (word & PI_STATUS_CLEAR_INTERRUPT) {
      clear_rcp_interrupt(pi->bus->vr4300, MI_INTR_PI);
      pi->regs[reg] &= ~PI_STATUS_INTERRUPT;
    }
  }

  else {
    if (pi->regs[PI_STATUS_REG] & PI_STATUS_IS_BUSY) {
      pi->regs[PI_STATUS_REG] |= PI_STATUS_ERROR;
      return 0;
    }

    pi->regs[reg] &= ~dqm;
    pi->regs[reg] |= word;

    if (reg == PI_CART_ADDR_REG)
      dd_pi_write(pi->bus->dd, word);

    else if (reg == PI_WR_LEN_REG) {
      if (pi->regs[PI_DRAM_ADDR_REG] == 0xFFFFFFFF) {
        pi->regs[PI_STATUS_REG] &= ~PI_STATUS_DMA_BUSY;
        pi->regs[PI_STATUS_REG] |= PI_STATUS_INTERRUPT;

        signal_rcp_interrupt(pi->bus->vr4300, MI_INTR_PI);
        return 0;
      }

      pi->bytes_to_copy = (pi->regs[PI_WR_LEN_REG] & 0xFFFFFF) + 1;
      pi->counter = pi->bytes_to_copy / 2 + 100; // Assume ~2 bytes/clock?
      pi->regs[PI_STATUS_REG] |= PI_STATUS_DMA_BUSY; // I'm busy!
      pi->is_dma_read = false;
    }

    else if (reg == PI_RD_LEN_REG) {
      if (pi->regs[PI_DRAM_ADDR_REG] == 0xFFFFFFFF) {
        pi->regs[PI_STATUS_REG] &= ~PI_STATUS_DMA_BUSY;
        pi->regs[PI_STATUS_REG] |= PI_STATUS_INTERRUPT;

        signal_rcp_interrupt(pi->bus->vr4300, MI_INTR_PI);
        return 0;
      }

      pi->bytes_to_copy = (pi->regs[PI_RD_LEN_REG] & 0xFFFFFF) + 1;
      pi->counter = pi->bytes_to_copy / 2 + 100; // Assume ~2 bytes/clock?
      pi->regs[PI_STATUS_REG] |= PI_STATUS_DMA_BUSY; // I'm busy!
      pi->is_dma_read = true;
    }
  }

  return 0;
}

// mapped read of flashram
int read_flashram(void *opaque, uint32_t address, uint32_t *word) {
  struct pi_controller *pi = (struct pi_controller *) opaque;
  if (pi->flashram.data == NULL)
    return -1;

  *word = pi->flashram.status >> 32;
  return 0;
}

// mapped write of flashram, commands
int write_flashram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct pi_controller *pi = (struct pi_controller *) opaque;

  if (pi->flashram.data == NULL) {
    debug("write to FlashRAM but no FlashRAM present\n");
    return 1;
  }

  if (address == 0x08000000) {
    debug("write to flash status, ignored");
    return 0;
  }

  switch (word >> 24) {
    case 0x4B: // set erase offset
      pi->flashram.offset = (word & 0xFFFF) * 128;
      break;

    case 0x78: // erase
      pi->flashram.mode = FLASHRAM_ERASE;
      pi->flashram.status = 0x1111800800C20000LL;
      break;

    case 0xA5: // set write offset
      pi->flashram.offset = (word & 0xFFFF) * 128;
      pi->flashram.status = 0x1111800400C20000LL;
      break;

    case 0xB4: // write
      pi->flashram.mode = FLASHRAM_WRITE;
      break;

    case 0xD2: // execute
      // TODO bounds checks
      if (pi->flashram.mode == FLASHRAM_ERASE)
        memset(pi->flashram.data + pi->flashram.offset, 0xFF, 0x80);

      else if (pi->flashram.mode == FLASHRAM_WRITE)
        memcpy(pi->flashram.data + pi->flashram.offset,
            pi->bus->ri->ram + pi->flashram.rdram_pointer, 0x80);

      break;

    case 0xE1: // status
      pi->flashram.mode = FLASHRAM_STATUS;
      pi->flashram.status = 0x1111800100C20000LL;
      break;

    case 0xF0: // read
      pi->flashram.mode = FLASHRAM_READ;
      pi->flashram.status = 0x11118004F0000000LL;
      break;

    default:
      debug("Unknown flashram command %08x\n", word);
  }

  return 0;
}

// Mapped read of SRAM
int read_sram(void *opaque, uint32_t address, uint32_t *word) {
  fprintf(stderr, "SRAM read\n");
  return 0;
}

// Mapped write of SRAM
int write_sram(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  fprintf(stderr, "SRAM write\n");
  return 0;
}
