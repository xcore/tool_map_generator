// Copyright (c) 2013, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#include "map_generator.h"
#include <iostream>
#include <cassert>

using namespace map_generator;

namespace {
  enum Opcode {
    LD8U = 0x11,
    LD16S = 0x10,
    LDW = 0x09,
    LDWI = 0x01,
    STWI = 0x00,
    ST8 = 0x119,
    ST16 = 0x109,
    STW = 0x009
  };

  enum Encoding {
    F3R,
    F2RUS,
    FL3R,
  };

  enum ShortType {
    SHORT_LD8U,
    SHORT_LD16S,
    SHORT_LDW,
    SHORT_STW,
    SHORT_PREFIX
  };

  enum LongType {
    LONG_ST8,
    LONG_ST16,
    LONG_STW,
  };

  enum MapType {
    MAP_SHORT,
    MAP_LONG,
    MAP_REG
  };
}

static struct {
  Opcode opcode;
  Encoding encoding;
  ShortType shortType;
  LongType longType;
} instructions[] = {
  { LD8U, F3R, SHORT_LD8U },
  { LD16S, F3R, SHORT_LD16S },
  { LDW, F3R, SHORT_LDW },
  { LDWI, F2RUS, SHORT_LDW },
  { STWI, F2RUS, SHORT_STW },
  { ST8, FL3R, SHORT_PREFIX, LONG_ST8 },
  { ST16, FL3R, SHORT_PREFIX, LONG_ST16 },
  { STW, FL3R, SHORT_PREFIX, LONG_STW }
};

static int16_t encode3r(unsigned opc, unsigned op0, unsigned op1, unsigned op2)
{
  uint16_t value = 0;
  value |= op2 & 0x3;
  value |= (op1 & 0x3) << 2;
  value |= (op0 & 0x3) << 4;
  value |= (op0 & 0x3) << 4;
  value |= ((op0 >> 2) * 9 + (op1 >> 2) * 3 + (op2 >> 2)) << 6;
  value |= opc << 11;
  return value;
}

static int32_t encodel3r(unsigned opc, unsigned op0, unsigned op1,
                         unsigned op2)
{
  uint16_t low = 0;
  uint16_t high = 0;
  low |= op2 & 0x3;
  low |= (op1 & 0x3) << 2;
  low |= (op0 & 0x3) << 4;
  low |= (op0 & 0x3) << 4;
  low |= ((op0 >> 2) * 9 + (op1 >> 2) * 3 + (op2 >> 2)) << 6;
  low |= 0x1f /* 0b11111 */ << 11;
  high |= opc & 0xf;
  high |= 0x7e /* 0b1111110 */ << 4;
  high |= (opc >> 4) << 11;
  return low | (high << 16);
}

template <typename T, size_t N>
static size_t array_size(const T (&a)[N]) {
  return N;
}

static bool isShort(Encoding encoding)
{
  switch (encoding) {
  case F3R:
  case F2RUS:
    return true;
  case FL3R:
    return false;
  }
}

static IntMap load_store_map(MapType map_type)
{
  IntMap map;
  for (unsigned i = 0; i != array_size(instructions); ++i) {
    const auto &instr = instructions[i];
    auto opcode = instr.opcode;
    if (map_type == MAP_LONG && isShort(instr.encoding))
      continue;
    for (unsigned op0 = 0; op0 != 12; ++op0) {
      for (unsigned op1 = 0; op1 != 12; ++op1) {
        for (unsigned op2 = 0; op2 != 12; ++op2) {
          uint32_t value;
          switch (instr.encoding) {
          case F2RUS:
          case F3R:
            value = encode3r(opcode, op0, op1, op2);
            break;
          case FL3R:
            value = encodel3r(opcode, op0, op1, op2);
            break;
          }
          uint32_t type;
          switch (map_type) {
          case MAP_SHORT:
            value &= 0xffff;
            type = instr.shortType;
            break;
          case MAP_REG:
            value &= 0xffff;
            type = op0;
            break;
          case MAP_LONG:
            value >>= 16;
            type = instr.longType;
            break;
          }
          auto res = map.insert(std::make_pair(value, type));
          assert(res.second || res.first->second == type);
        }
      }
    }
  }
  return map;
}

int main()
{
  Steps steps = generate(load_store_map(MAP_SHORT), 10);
  steps.print(std::cout, "map_short");
  CodeCost cost = steps.compute_cost();
  std::cerr << "Number of bytes: " << cost.bytes << '\n';
  std::cerr << "Number of cycles: " << cost.cycles << '\n';

  steps = generate(load_store_map(MAP_LONG), 10);
  steps.print(std::cout, "map_long");
  cost = steps.compute_cost();
  std::cerr << "Number of bytes: " << cost.bytes << '\n';
  std::cerr << "Number of cycles: " << cost.cycles << '\n';

  steps = generate(load_store_map(MAP_REG), 10);
  steps.print(std::cout, "map_reg");
  cost = steps.compute_cost();
  std::cerr << "Number of bytes: " << cost.bytes << '\n';
  std::cerr << "Number of cycles: " << cost.cycles << '\n';
  return 0;
}
