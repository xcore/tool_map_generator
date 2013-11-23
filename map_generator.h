#ifndef _map_generator_h_
#define _map_generator_h_

#include <unordered_map>
#include <vector>
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

    void print(std::ostream &);
    CodeCost compute_cost();
  };

  typedef std::unordered_map<uint32_t,uint32_t> IntMap;

  Steps generate(const IntMap &map, unsigned max_steps = 1);
}

#endif // _map_generator_h_
