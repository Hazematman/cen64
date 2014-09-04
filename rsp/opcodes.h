//
// rsp/opcodes.h: RSP opcode types and info.
//
// CEN64: Cycle-Accurate Nintendo 64 Simulator.
// Copyright (C) 2014, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#ifndef __rsp_opcodes_h__
#define __rsp_opcodes_h__
#include "common.h"
#include "rsp/rsp.h"

enum rsp_opcode_id {
#define X(op) RSP_OPCODE_##op,
#include "rsp/opcodes.md"
  NUM_RSP_OPCODES
#undef X
};

enum rsp_vector_opcode_id {
#define X(op) RSP_OPCODE_##op,
#include "rsp/vector_opcodes.md"
  NUM_RSP_VECTOR_OPCODES
#undef X
};

struct rsp;
typedef int (*rsp_function)(struct rsp *,
  uint32_t, uint64_t, uint64_t);

typedef int (*rsp_vector_function)(struct rsp *, uint32_t,
  unsigned, rsp_vect_t vs, rsp_vect_t vt, rsp_vect_t vd);

extern const rsp_function rsp_function_table[NUM_RSP_OPCODES];
extern const char *rsp_opcode_mnemonics[NUM_RSP_OPCODES];
extern const rsp_vector_function rsp_vector_function_table[NUM_RSP_VECTOR_OPCODES];
extern const char *rsp_vector_opcode_mnemonics[NUM_RSP_VECTOR_OPCODES];

#endif

