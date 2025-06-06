#include "page_table.h"
#include <limits>

const uint64_t PageTable::INVALID_VA = std::numeric_limits<uint64_t>::max();
const PageTable::PFN PageTable::INVALID_PFN = 0;

PageTable::PageTable(uint64_t max_pfn_)
    : max_pfn(max_pfn_ + 1), next_free_pfn(1),
      l3_table(PT_LEVEL_ENTRIES),
      pfn_in_use(max_pfn_ + 1, false),
      pfn_to_va_map(max_pfn_ + 1, INVALID_VA)
{
    pfn_in_use[0] = true;
    pfn_to_va_map[0] = INVALID_VA;
}

PageTable::PFN PageTable::allocate_pfn_internal() {
    next_free_pfn = (next_free_pfn >= max_pfn - 1) ? 1 : next_free_pfn + 1;
    PFN candidate = next_free_pfn;

    if (pfn_in_use[candidate]) {
        uint64_t old_va = pfn_to_va_map[candidate];
        if (old_va != INVALID_VA)
            unmap_page_internal(old_va);
    }

    pfn_in_use[candidate] = true;
    return candidate;
}

uint64_t PageTable::l3_index(uint64_t va) const {
    return (va >> (PAGE_SHIFT + 2 * PT_LEVEL_BITS)) & (PT_LEVEL_ENTRIES - 1);
}

uint64_t PageTable::l2_index(uint64_t va) const {
    return (va >> (PAGE_SHIFT + PT_LEVEL_BITS)) & (PT_LEVEL_ENTRIES - 1);
}

uint64_t PageTable::l1_index(uint64_t va) const {
    return (va >> PAGE_SHIFT) & (PT_LEVEL_ENTRIES - 1);
}

PageTable::PFN PageTable::map_page_internal(uint64_t va) {
    PFN pfn = allocate_pfn_internal();

    auto &l2_table = l3_table[l3_index(va)];
    if (l2_table.empty())
        l2_table.resize(PT_LEVEL_ENTRIES);

    auto &l1_table = l2_table[l2_index(va)];
    if (l1_table.empty())
        l1_table.resize(PT_LEVEL_ENTRIES, INVALID_PFN);

    l1_table[l1_index(va)] = pfn;
    pfn_to_va_map[pfn] = va;

    return pfn;
}

bool PageTable::unmap_page_internal(uint64_t va) {
    auto &l2_table = l3_table[l3_index(va)];
    if (l2_table.empty()) return false;

    auto &l1_table = l2_table[l2_index(va)];
    if (l1_table.empty()) return false;

    PFN &entry = l1_table[l1_index(va)];
    if (entry == INVALID_PFN) return false;

    pfn_in_use[entry] = false;
    pfn_to_va_map[entry] = INVALID_VA;
    entry = INVALID_PFN;

    return true;
}

bool PageTable::lookup_pfn_internal(uint64_t va, PFN &out_pfn) const {
    const auto &l2_table = l3_table[l3_index(va)];
    if (l2_table.empty()) return false;

    const auto &l1_table = l2_table[l2_index(va)];
    if (l1_table.empty()) return false;

    out_pfn = l1_table[l1_index(va)];
    return out_pfn != INVALID_PFN;
}

PageTable::PFN PageTable::map_page(uint64_t va) {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_page_internal(va);
}

bool PageTable::unmap_page(uint64_t va) {
    std::lock_guard<std::mutex> lock(mutex_);
    return unmap_page_internal(va);
}

bool PageTable::lookup_pfn(uint64_t va, PFN &out_pfn) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lookup_pfn_internal(va, out_pfn);
}

PageTable::PFN PageTable::get_or_map_page(uint64_t va) {
    std::lock_guard<std::mutex> lock(mutex_);
    PFN pfn;
    if (lookup_pfn_internal(va, pfn))
        return pfn;
    return map_page_internal(va);
}