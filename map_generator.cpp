#include "map_generator.h"
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <ostream>
#include <numeric>
#include <cassert>

using namespace map_generator;

static uint32_t crc32(uint32_t checksum, uint32_t data, uint32_t poly)
{
  for (unsigned i = 0; i < 32; i++) {
    int xorBit = (checksum & 1);

    checksum >>= 1;

    if (xorBit)
      checksum = checksum ^ poly;
  }
  return data ^ checksum;
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

class map_generator::Step {
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

namespace {
  typedef std::vector<IntMap::value_type> IntPairs;

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

  template <typename T>
  uint32_t get_lookup_table_size(const T &map)
  {
    uint32_t size = 0;
    for (const auto &entry : map) {
      size = std::max(size, entry.first);
    }
    return size + 1;
  }

  class LookupTableStep : public Step {
    IntMap table;
    DataType type;
  public:
    LookupTableStep(IntMap table, DataType type) :
    table(std::move(table)), type(type) {}
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
    poly(poly), table(std::move(table)), type(type),
    collision_sentinal(sentinal) {}
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
  std::vector<std::pair<uint32_t,uint32_t>> entries(begin(table), end(table));
  std::sort(begin(entries), end(entries));
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

namespace {
  template <typename T> bool
  apply_hash_to_map(const IntPairs &map, T hash,
                    const HashCost &best_cost,
                    uint32_t collision_sentinal,
                    IntMap &result,
                    HashCost &cost)
  {
    cost = HashCost();
    result.clear();
    for (const auto &entry : map) {
      uint32_t value = hash(entry.first);
      auto res = result.insert(std::make_pair(value, entry.second));
      if (res.second || res.first->second == entry.second ||
          res.first->second == collision_sentinal)
        continue;
      // Handle collisions.
      ++cost.num_collisions;
      if (cost >= best_cost)
        return false;
      res.first->second = collision_sentinal;
    }
    cost.table_size = get_lookup_table_size(result);
    if (cost >= best_cost)
      return false;
    return true;
  }

  template <typename T>
  bool apply_hash_to_map(const IntPairs &map, T hash,
                         IntMap &result)
  {
    HashCost dummy;
    auto cost = HashCost::max();
    cost.num_collisions = 0;
    return apply_hash_to_map(map, hash, cost, 0, result, dummy);
  }
}

static uint32_t find_unused_value(const std::vector<uint32_t> &values)
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
  IntPairs map_entries(begin(map), end(map));
  std::vector<uint32_t> unique_values;
  unique_values.reserve(map_entries.size());
  for (const auto &entry : map_entries) {
    unique_values.push_back(entry.second);
  }
  std::sort(unique_values.begin(), unique_values.end());
  unique_values.erase(std::unique(unique_values.begin(), unique_values.end()),
                      unique_values.end());
  unsigned num_unique_values = unique_values.size();
  uint32_t collision_sentinal;
  if (allow_conflicts) {
    collision_sentinal = find_unused_value(unique_values);
    ++num_unique_values;
  }
  uint32_t best = 0;
  HashCost best_cost = HashCost::max();
  if (!allow_conflicts) {
    best_cost.num_collisions = 0;
    best_cost.table_size = get_lookup_table_size(map_entries);
    if (best_cost.table_size <= 4)
      return;
    best_cost.table_size -= 4;
  }
  IntMap tmp;
  tmp.reserve(map_entries.size());
  unsigned min_width = log2_ceil(num_unique_values) - 1;
  for (unsigned width = min_width; width < 32; width++) {
    // Don't check polynomials of higher widths - we are unlikely to get any
    // further improvement.
    if (best != 0 || (1 << width) > best_cost.table_size)
      break;
    for (uint32_t poly = 1 << width; poly < (1 << (width + 1));
         poly++) {
      auto hash = [=](uint32_t x) {
        return crc32(x, poly, poly);
      };
      HashCost cost;
      if (apply_hash_to_map(map_entries, hash, best_cost, collision_sentinal,
                            tmp, cost)) {
        best = poly;
        best_cost = cost;
      }
    }
  }
  if (best == 0)
    return;
  auto hash = [=](uint32_t x) {
    return crc32(x, best, best);
  };
  if (allow_conflicts) {
    HashCost dummy;
    apply_hash_to_map(map_entries, hash, HashCost::max(), collision_sentinal,
                      tmp, dummy);
    DataType data_type = pick_datatype(tmp);
    steps.push_back(
      std::unique_ptr<Step>(
        new CrcLookupAndReturnStep(best, std::move(tmp), data_type,
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
    apply_hash_to_map(map_entries, hash, map);
    steps.push_back(std::unique_ptr<Step>(new CrcStep(best)));
  }
}

Steps::Steps(std::vector<std::unique_ptr<Step>> steps) :
  steps(std::move(steps)) {
}

Steps::~Steps() {
}

Steps::Steps(Steps &&other) : steps(std::move(other.steps)) {
}

void Steps::print(std::ostream &out)
{
  out << "uint32_t map(uint32_t x)\n";
  out << "{\n";
  for (const auto &step : steps) {
    step->print(out);
  }
  out << "  return x;\n";
  out << "}\n";
}

CodeCost Steps::compute_cost()
{
  CodeCost cost;
  for (const auto &step : steps) {
    cost += step->compute_cost();
  }
  return cost;
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

Steps map_generator::generate(const IntMap &map, unsigned max_steps)
{
  std::vector<std::unique_ptr<Step>> steps;
  auto lookup = map;
  for (unsigned i = 0; i != max_steps; i++) {
    bool allow_collisions = i + 1 != max_steps;
    reduce_using_crc(lookup, allow_collisions, steps);
  }
  DataType data_type = pick_datatype(lookup);
  steps.push_back(
    std::unique_ptr<Step>(
      new LookupTableStep(std::move(lookup), data_type)));
  assert(validate(map, steps));
  return Steps(std::move(steps));
}
