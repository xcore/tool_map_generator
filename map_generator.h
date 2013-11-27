// Copyright (c) 2013, XMOS Ltd., All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>

#ifndef _map_generator_h_
#define _map_generator_h_

#include <unordered_map>
#include <vector>
#include <string>
#include <iosfwd>

namespace map_generator {
  class Step;

  struct CodeCost {
    unsigned bytes;
    unsigned cycles;
    CodeCost() : bytes(0), cycles(0) {}
    CodeCost(unsigned bytes, unsigned cycles) : bytes(bytes), cycles(cycles) {}
  };

  class Steps {
    std::vector<std::unique_ptr<Step>> steps;
  public:
    Steps(std::vector<std::unique_ptr<Step>> steps);
    ~Steps();
    Steps(Steps &&);
    Steps &operator=(Steps &&);

    /// Emit a C function implementing the steps.
    void print(std::ostream &, const std::string &name = "map");
    /// Estimate the cost of the sequence of steps.
    CodeCost compute_cost();
  };

  typedef std::unordered_map<uint32_t,uint32_t> IntMap;

  /// Generate a sequence of steps implementing the specified map.
  /// \param max_steps The maximum number of steps to use.
  Steps generate(const IntMap &map, unsigned max_steps = 1);
}

#endif // _map_generator_h_
