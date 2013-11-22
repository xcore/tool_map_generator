#include <algorithm>
#include <bitset>
#include <climits>
#include <vector>
#include <unordered_map>
#include <set>
#include <cstdlib>
#include <iostream>
#include <initializer_list>
#include <random>
#include <numeric>
#include <cassert>

namespace {
  template <typename T> uint32_t crc(uint32_t checksum, T data, uint32_t poly) {
    for (unsigned i = 0; i < sizeof(T) * CHAR_BIT; i++) {
      int xorBit = (checksum & 1);
      
      checksum  = (checksum >> 1) | ((data & 1) << 31);
      data = data >> 1;
      
      if (xorBit)
        checksum = checksum ^ poly;
    }
    return checksum;
  }
}

static uint32_t crc32(uint32_t checksum, uint32_t data, uint32_t poly)
{
  return crc<uint32_t>(checksum, data, poly);
}

static unsigned clz(uint32_t x)
{
  if (x == 0)
    return 32;
  return __builtin_clz(x);
}

static unsigned log2_ceil(uint32_t x)
{
  return 32 - clz(x - 1);
}

namespace {
  typedef std::unordered_map<uint32_t,uint32_t> IntMap;

  struct CodeCost {
    unsigned bytes;
    unsigned cycles;
    CodeCost() : bytes(0), cycles(0) {}
    CodeCost(unsigned bytes, unsigned cycles) : bytes(bytes), cycles(cycles) {}
  };

  class Step {
  public:
    enum Result {
      DONE,
      NEXT,
      ERROR
    };
    virtual Result apply(uint32_t &) const = 0;
    virtual void print(std::ostream &) const = 0;
    virtual CodeCost compute_cost() const = 0;
  };

  enum class DataType {
    S8,
    U8,
    S16,
    U16,
    S32,
    U32
  };

  static bool is_signed(DataType type)
  {
    switch (type) {
    case DataType::S8:
    case DataType::S16:
    case DataType::S32:
      return true;
    case DataType::U8:
    case DataType::U16:
    case DataType::U32:
      return false;
    }
  }

  static unsigned get_size(DataType type)
  {
    switch (type) {
    case DataType::S8:
    case DataType::U8:
      return 1;
    case DataType::S16:
    case DataType::U16:
      return 2;
    case DataType::S32:
    case DataType::U32:
      return 4;
    }
  }

  static uint32_t get_lookup_table_size(const IntMap &map)
  {
    uint32_t size = 0;
    for (const auto &entry : map) {
      size = std::max(size, entry.first);
    }
    return size;
  }

  class LookupTableStep : public Step {
    IntMap table;
    DataType type;
  public:
    LookupTableStep(IntMap table, DataType type) :
      table(table), type(type) {}
    Result apply(uint32_t &x) const override {
      auto match = table.find(x);
      if (match == table.end())
        return ERROR;
      x = match->second;
      return NEXT;
    }
    void print(std::ostream &out) const override;
    CodeCost compute_cost() const override {
      unsigned num_bytes = get_lookup_table_size(table) * get_size(type);
      num_bytes += 4 + 2;
      return CodeCost(num_bytes, 2);
    }
  };

  class CrcLookupAndReturnStep : public Step {
    uint32_t poly;
    IntMap table;
    DataType type;
    uint32_t collision_sentinal;
  public:
    CrcLookupAndReturnStep(uint32_t poly, IntMap table,
                           DataType type, uint32_t sentinal) :
      poly(poly), table(table), type(type), collision_sentinal(sentinal) {}
    Result apply(uint32_t &x) const override {
      uint32_t tmp = crc32(x, poly, poly);
      auto match = table.find(tmp);
      if (match == table.end())
        return ERROR;
      if (match->second != collision_sentinal) {
        x = match->second;
        return DONE;
      }
      return NEXT;
    }
    void print(std::ostream &out) const override;
    CodeCost compute_cost() const override {
      unsigned num_bytes = get_lookup_table_size(table) * get_size(type);
      num_bytes += 4 + 2 + 4 + 2 + 2;
      return CodeCost(num_bytes, 5);
    }
  };

  class CrcStep : public Step {
    uint32_t poly;
  public:
    CrcStep(uint32_t poly) : poly(poly) {}
    Result apply(uint32_t &x) const override {
      x = crc32(x, poly, poly);
      return NEXT;
    }
    void print(std::ostream &out) const override {
      out << "  crc32(x, 0x" << poly << ", 0x" << poly << ");\n";
    }
    CodeCost compute_cost() const override {
      return CodeCost(4 + 2, 2);
    }
  };

  struct HashCost {
    unsigned num_collisions;
    uint32_t table_size;
    HashCost() : num_collisions(0), table_size(0) {}
    HashCost(unsigned num_collisions, uint32_t table_size) :
      num_collisions(num_collisions), table_size(table_size) {}
    static HashCost max() {
      return HashCost(std::numeric_limits<unsigned>::max(),
                      std::numeric_limits<uint32_t>::max());
    }
  };
}

static CodeCost &operator+=(CodeCost &a, const CodeCost b)
{
  a.bytes += b.bytes;
  a.cycles += b.cycles;
  return a;
}

static bool operator<(const HashCost &a, const HashCost &b)
{
  if (a.num_collisions != b.num_collisions)
    return a.num_collisions < b.num_collisions;
  return a.table_size < b.table_size;
}

static bool operator>=(const HashCost &a, const HashCost &b)
{
  return !(a < b);
}

static const char *get_type_name(DataType type)
{
  switch (type) {
  case DataType::S8:
    return "int8_t";
  case DataType::U8:
    return "uint8_t";
  case DataType::S16:
    return "int16_t";
  case DataType::U16:
    return "uint16_t";
  case DataType::S32:
    return "int32_t";
  case DataType::U32:
    return "uint32_t";
  }
}

static void
print_lookup_table(std::ostream &out, const IntMap &table,
                   DataType type)
{
  out << "    static const " << get_type_name(type) << " lookup[] = {\n";
  unsigned i = 0;
  std::vector<std::pair<uint32_t,uint32_t>> entries(table.begin(), table.end());
  std::sort(entries.begin(), entries.end());
  for (const auto &entry : entries) {
    for (; i < entry.first; i++) {
      out << "      0, /* " << i << " -> unused */\n";
    }
    if (is_signed(type)) {
      out << "      " << static_cast<int32_t>(entry.second) << ',';
    } else {
      out << "      " << entry.second << ',';
    }
    out << " /* " << entry.first << " -> " << entry.second << " */\n";
    i++;
  }
  out << "    };\n";
}

void LookupTableStep::print(std::ostream &out) const
{
  out << "  {\n";
  print_lookup_table(out, table, type);
  out << "    x = lookup[x];\n";
  out << "  };\n";
}

void CrcLookupAndReturnStep::print(std::ostream &out) const
{
  out << "  {\n";
  print_lookup_table(out, table, type);
  out << "    uint32_t tmp = x;\n";
  out << "    crc32(tmp, 0x" << poly << ", 0x" << poly << ");\n";
  out << "    tmp = lookup[tmp];\n";
  out << "    if (tmp != " << collision_sentinal << ") {\n";
  out << "      return tmp;\n";
  out << "    }\n";
  out << "  };\n";
}

static IntMap readmap()
{
  IntMap map{
    {0, 0},
    {1, 2},
    {2, 3},
    {100, 4},
    {1897722, 5},
  };
  return map;
}

namespace {
  template <typename T> bool
  apply_hash_to_map(const IntMap &map, T hash,
                    const HashCost &best_cost,
                    uint32_t collision_sentinal,
                    IntMap &result,
                    HashCost &cost)
  {
    cost = HashCost();
    IntMap reduced;
    for (const auto &entry : map) {
      uint32_t value = hash(entry.first);
      auto res = reduced.insert(std::make_pair(value, entry.second));
      if (res.second || res.first->second == entry.second ||
          res.first->second == collision_sentinal)
        continue;
      // Handle collisions.
      ++cost.num_collisions;
      if (cost >= best_cost)
        return false;
      res.first->second = collision_sentinal;
    }
    cost.table_size = get_lookup_table_size(reduced);
    if (cost >= best_cost)
      return false;
    result = std::move(reduced);
    return true;
  }
  
  template <typename T>
  bool apply_hash_to_map(const IntMap &map, T hash,
                         IntMap &result)
  {
    HashCost dummy;
    auto cost = HashCost::max();
    cost.num_collisions = 0;
    return apply_hash_to_map(map, hash, cost, 0, result, dummy);
  }

  template <typename T>
  bool apply_hash_to_map_inplace(IntMap &map, T hash)
  {
    return apply_hash_to_map(map, hash, map);
  }

  template <typename T> std::bitset<32>
  reduce_using_bitop(IntMap &map, T op)
  {
    std::bitset<32> mask;
    for (int bit = 31; bit >= 0; bit--) {
      std::bitset<32> candidate_mask = mask;
      candidate_mask.set(bit);
      if (apply_hash_to_map_inplace(map, [=](uint32_t x) {
        return op(x, candidate_mask.to_ulong());
      })) {
        mask = candidate_mask;
      }
    }
    return mask;
  }
}

static std::bitset<32> reduce_using_and(IntMap &map)
{
  std::bitset<32> mask =
    reduce_using_bitop(map, [](uint32_t a, uint32_t b) { return a & ~b; });
  return mask.flip();
}

static uint32_t find_unused_value(const std::set<uint32_t> &values)
{
  unsigned i = 0;
  for (auto value : values) {
    if (value != i)
      return i;
    ++i;
  }
  return values.size();
}

static DataType pick_datatype(const IntMap &lookup)
{
  uint32_t max_u = 0;
  int32_t min_s = 0;
  int32_t max_s = 0;
  for (const auto &entry : lookup) {
    uint32_t value_u = entry.second;
    int32_t value_s = static_cast<int32_t>(value_u);
    max_u = std::max(max_u, value_u);
    max_s = std::max(max_s, value_s);
    min_s = std::min(min_s, value_s);
  }
  // Prefer unsigned 8bit loads signed as there is no 8bit signed load.
  if (max_u <= std::numeric_limits<uint8_t>::max())
    return DataType::U8;
  if (min_s >= std::numeric_limits<int8_t>::min() &&
      max_s <= std::numeric_limits<int8_t>::max())
    return DataType::S8;
  // Prefer signed 16bit loads as there is no 16bit unsigned load.
  if (min_s >= std::numeric_limits<int16_t>::min() &&
      max_s <= std::numeric_limits<int16_t>::max())
    return DataType::S16;
  if (max_u <= std::numeric_limits<uint16_t>::max())
    return DataType::U16;
  return DataType::U32;
}

static void
reduce_using_crc(IntMap &map,
                 bool allow_conflicts,
                 std::vector<std::unique_ptr<Step>> &steps)
{
  std::set<uint32_t> unique_values;
  for (const auto &entry : map) {
    unique_values.insert(entry.second);
  }
  uint32_t collision_sentinal;
  if (allow_conflicts) {
    collision_sentinal = find_unused_value(unique_values);
    unique_values.insert(collision_sentinal);
  }
  uint32_t best = 0;
  HashCost best_cost = HashCost::max();
  if (!allow_conflicts) {
    best_cost.num_collisions = 0;
    best_cost.table_size = get_lookup_table_size(map);
    if (best_cost.table_size <= 4)
      return;
    best_cost.table_size -= 4;
  }
  unsigned min_width = log2_ceil(unique_values.size()) - 1;
  for (unsigned width = min_width; width < 32; width++) {
    for (uint32_t poly = 1 << width; poly < (1 << (width + 1));
         poly++) {
      IntMap tmp;
      auto hash = [=](uint32_t x) {
        return crc32(x, poly, poly);
      };
      HashCost cost;
      if (apply_hash_to_map(map, hash, best_cost, collision_sentinal, tmp,
                            cost)) {
        best = poly;
        best_cost = cost;
      }
    }
    // Don't check polynomials of higher widths - we are unlikely to get any
    // further improvement.
    if (best != 0)
      break;
  }
  if (best == 0)
    return;
  auto hash = [=](uint32_t x) {
    return crc32(x, best, best);
  };
  if (allow_conflicts) {
    HashCost dummy;
    IntMap lookup;
    apply_hash_to_map(map, hash, HashCost::max(), collision_sentinal, lookup,
                      dummy);
    DataType data_type = pick_datatype(lookup);
    steps.push_back(
      std::unique_ptr<Step>(
        new CrcLookupAndReturnStep(best, std::move(lookup), data_type,
                                   collision_sentinal)
      )
    );
    // Build map containing only conflicting keys.
    IntMap remaining;
    for (const auto &entry : map) {
      uint32_t x = entry.first;
      if (steps.back()->apply(x) != Step::DONE) {
        remaining.insert(entry);
      }
    }
    std::swap(map, remaining);
  } else {
    apply_hash_to_map_inplace(map, hash);
    steps.push_back(std::unique_ptr<Step>(new CrcStep(best)));
  }
}

static unsigned reduce_using_shift(IntMap &map)
{
  unsigned shift;
  for (shift = 0; shift < 32; shift++) {
    if (!apply_hash_to_map_inplace(map, [](uint32_t x) {
      return x >> 1;
    })) {
      break;
    }
  }
  return shift;
}

static void print(const std::vector<std::unique_ptr<Step>> &steps)
{
  std::cout << "uint32_t map(uint32_t x)\n";
  std::cout << "{\n";
  for (const auto &step : steps) {
    step->print(std::cout);
  }
  std::cout << "  return x;\n";
  std::cout << "}\n";
}

static void stats(const std::vector<std::unique_ptr<Step>> &steps)
{
  CodeCost cost;
  for (const auto &step : steps) {
    cost += step->compute_cost();
  }
  std::cerr << "Number of bytes: " << cost.bytes << '\n';
  std::cerr << "Number of cycles: " << cost.cycles << '\n';
}

static bool validate(const IntMap &map,
                     const std::vector<std::unique_ptr<Step>> &steps)
{
  for (const auto &entry : map) {
    uint32_t value = entry.first;
    for (const auto &step : steps) {
      Step::Result result = step->apply(value);
      if (result == Step::DONE)
        break;
      if (result == Step::ERROR)
        return false;
      assert(result == Step::NEXT);
    }
    if (value != entry.second)
      return false;
  }
  return true;
}

static IntMap testmap()
{
  IntMap map;
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution;
  auto rand = std::bind(distribution, generator);
  for (unsigned i = 0; i < 1000; i++) {
    map.insert(std::make_pair(rand(), rand()));
  }
  return map;
}

static IntMap testmap2()
{
  IntMap map;
  for (unsigned i = 0; i < 256; i++) {
    unsigned unzipped = 0;
    for (unsigned j = 0; j < 8; j++) {
      if (i & (1 << j)) {
        unzipped |= 1 << (j * 4);
      }
    }
    map.insert(std::make_pair(unzipped, i));
  }
  return map;
}

int main()
{
  std::vector<std::unique_ptr<Step>> steps;
  const unsigned max_stages = 8;
  const auto map = testmap();
  auto lookup = map;
  for (unsigned i = 0; i != max_stages; i++) {
    bool allow_collisions = i + 1 != max_stages;
    reduce_using_crc(lookup, allow_collisions, steps);
  }
  DataType data_type = pick_datatype(lookup);
  steps.push_back(
    std::unique_ptr<Step>(
      new LookupTableStep(std::move(lookup), data_type)));
  print(steps);
  if (!validate(map, steps)) {
    std::cerr << "Failed validation\n";
    std::exit(1);
  }
  stats(steps);
  return 0;
}
