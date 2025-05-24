#ifndef _MC_H_
#define _MC_H_

#include "config.h"
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include <string>
#include "stats.h"
#include "g_std/g_unordered_map.h"
#include "g_std/g_vector.h"
#include<math.h>
#include<stdio.h>
#include<iostream>
#include <array>
#include <cstdint>

#define MAX_STEPS 10000

enum ReqType
{
	LOAD = 0,
	STORE
};


enum Scheme
{
   AlloyCache,
   UnisonCache,
   HMA,
   HybridCache,
   NoCache,
   CacheOnly,
   Tagless,
   BasicCache,
   SDCache,
   Trimma,
};

using PhysicalAddr = Address;
using DeviceAddr = Address;
using BlockID = uint32_t;
using BitVector = uint32_t;

/**
 * @brief iRT 为不同的集合（set）使用独立的树结构;
 * 		  Trimma核心数据结构，Radix Tree (per set); ​Radix核心理念​​：将键值按比特位分割，逐层映射到树节点；
 * 		  Trimma 使用 11 位 tag chunk 来构建一个 2048 叉的 radix tree；
 * 		  一个简单的两级 iRT 结构就足以支持多数情况，进一步增加层级并不能带来更多空间节省
 * @author Jiahao Lu @ XMU
 * @attention 对于一个PA，其索引位首先选择该地址所在集合的 tag 根节点; tag_roots_
 * 			  该根节点指向对应的多级重映射表的根 //
 * 			  PA中的 tag 位被划分为多个部分，用于逐级遍历重映射表，直到定位到叶子节点，
 * 			  该节点存储重映射后的block ID，再与block offset拼接生成最终的DA。
 * 
 * 			  > iRT查找失败，则DA==PA （未被cache/migrate或未allocate）		  
 */
class iRT
{
public:
	struct iRNode{
		bool is_leaf;
		union{
			BitVector* allocated_bits; // 中间节点仅使用2048位向量（bit vector）来表示下一层是否已分配
			BlockID remapped_id; // 仅叶子节点存储重映射块 ID
		};
	};

	static constexpr uint32_t BITS_PER_LEVEL = 11;   // 每层11位
	static constexpr uint32_t CHILDREN_PER_NODE = 1 << BITS_PER_LEVEL; // 2048
	static constexpr uint32_t LEVELS = 2;             // 两级结构

	// 48位地址
	struct AddrLayout {
        static constexpr uint32_t SET_BITS = 11;      // 2048 sets
        static constexpr uint32_t TAG_BITS = BITS_PER_LEVEL * LEVELS; // 22位
        static constexpr uint32_t OFFSET_BITS = 12;   // 4KB块偏移
        static_assert(SET_BITS + TAG_BITS + OFFSET_BITS <= 48, "Address overflow");
        
        // 地址解码
        static uint32_t set(PhysicalAddr pa) { 
            return (pa >> (TAG_BITS + OFFSET_BITS)) & ((1 << SET_BITS)-1); 
        }
        static uint32_t tag(PhysicalAddr pa, int level) {
            return (pa >> (OFFSET_BITS + (LEVELS-level-1)*BITS_PER_LEVEL)) 
                   & ((1 << BITS_PER_LEVEL)-1);
        }
    };

	g_vector<iRNode> node_pool_;
	g_vector<uint32_t> tag_roots_; // 根节点索引
	static constexpr uint32_t INVALID_INDEX = ~0u;

 	iRT(int sets) : tag_roots_(sets, INVALID_INDEX) {
        // 预分配根节点（每个set一个）
        for(auto& root : tag_roots_) {
            root = allocate_node(false); // 中间节点
        }
    }

	/**
	 * @brief 根据物理地址进行多级查找。首先提取set_index，如果超出范围则返回恒等映射。
	 * 		  然后获取根节点的current_idx，如果是无效的，直接返回。
	 * 		  然后循环遍历LEVEL_SIZES的层级数减一，因为最后一级是叶节点。
	 * 		  在每个层级，检查节点是否是叶节点（可能提前终止循环），然后提取当前层级的slot。
	 * 		  检查该slot是否已经分配，如果没有，返回恒等映射;
	 * 		  否则计算子节点索引并继续查找。最后处理叶节点，合成设备地址。
	 * @example pa=0x12345678： 
	 * 		【假设如下】
	 * 			| 11-bit Set | 5-bit Level0 | 5-bit Level1 | 8-bit Block Offset |
	 * 			| (0x3FF)    | (0x1F)       | (0x1F)       | (0xFF)             |
	 * 			 0001 0010 0011 0100 0101 0110 0111 1000
	 * 			| 0001 0010 001 | 1 0100 | 0 1011 | ... | 0111 1000 |
	 * 		【提取流程】
	 * 			set_idx = 0001 0010 001 → 0x113
	 * 			→ tag_roots_[0x113]
	 * 			Level-0:1 0100 → 0x14
	 * 			Level-1:0 1011 → 0x0B
	 * 
	 */
	DeviceAddr translate(PhysicalAddr pa) const {
        const uint32_t set_idx = AddrLayout::set(pa);
        if(set_idx >= tag_roots_.size()) return pa;
        
        uint32_t current_idx = tag_roots_[set_idx];
        for(uint32_t level = 0; level < LEVELS-1; ++level) { // 仅遍历中间层
            if(current_idx >= node_pool_.size())return pa; // 越界保护
			const auto& node = node_pool_[current_idx];
            if(node.is_leaf) break; // 提前终止
            
            const uint32_t slot = AddrLayout::tag(pa, level);
            if(!check_bit(node.allocated_bits, slot)) 
                return pa;
                
            current_idx = node.allocated_bits[slot / 32] >> (slot % 32); // 简化版子节点索引
        }
        
        // 叶子节点处理
        const auto& leaf = node_pool_[current_idx];
        return leaf.is_leaf ? 
            (leaf.remapped_id << AddrLayout::OFFSET_BITS) | (pa & ((1<<AddrLayout::OFFSET_BITS)-1)) 
            : pa;
    }

private:
    // 位操作辅助函数
    bool check_bit(uint32_t* bits, uint32_t pos) const {
        return (bits[pos/32] >> (pos%32)) & 0x1;
    }
    
    /**
	 * @brief 节点分配
	 * @todo 处理内存对齐 ?
	 */
    uint32_t allocate_node(bool is_leaf) {
        const uint32_t idx = node_pool_.size();
        iRNode node;
        node.is_leaf = is_leaf;
        if(!is_leaf) {
            node.allocated_bits = new uint32_t[CHILDREN_PER_NODE/32](); // 64x32-bit
        }
        node_pool_.push_back(node);
        return idx;
    }
};


/**
 * 使用多级表结构的一个主要问题是查找延迟增加。
 * 一个具有 𝐿 级的重映射表在最坏情况下可能引入最多 𝐿 + 1 次的片外访问。
 * 就像操作系统页表依赖 TLB 来加速访问一样，重映射表也需要更高效的缓存机制。
 * 
 * 
 */

/**
 * @brief NonIdCache条目：非身份映射的设备地址
 */
class NonIdCacheEntry
{
public:
	uint32_t phy_tag;    // 物理地址标签
	uint64_t dev_addr;   // 设备地址
	bool valid;
};

/**
 * @brief 用于存储与传统方式相同的有效重映射项（非恒等映射）。
 */
class NonIdCache
{
public:
	struct NonIdCacheSet
	{
		g_vector<NonIdCacheEntry> ways; // 每个set 6 个ways
		g_vector<uint32_t> lru_value; // 替换策略就按LRU来吧

		NonIdCacheSet() : ways(6), lru_value(6, 0) { // 默认初始化valid为false
            for (auto& entry : ways) {
                entry = NonIdCacheEntry(); // 显式初始化
            }
        }
	};

	g_vector<NonIdCacheSet> sets; // 2048个

	NonIdCache() : sets(2048){}

	bool lookup(PhysicalAddr pa, DeviceAddr& da)
	{
		uint32_t set_idx = (pa >> 6) & 0x7FF; // 暂定11位索引，后续再改
		uint32_t tag = pa >> 17; // 剩余位作为tag

		NonIdCacheSet& target_set = sets[set_idx];
		for (int i = 0; i < 6; ++i) {
            if (target_set.ways[i].valid && 
                target_set.ways[i].phy_tag == tag) {
                // 更新LRU信息
                update_lru(target_set.lru_value, i);
                da = target_set.ways[i].dev_addr;
                return true;
            }
        }
        return false;
 	}

	void insert(PhysicalAddr pa, DeviceAddr da)
	{
		uint32_t set_idx = (pa >> 6) & 0x7FF;
		uint32_t tag = pa >> 17;

		NonIdCacheSet& target_set = sets[set_idx];
		int victim_way = find_lru_victim(target_set.lru_value);

		target_set.ways[victim_way] = {tag, da, true};
		update_lru(target_set.lru_value, victim_way);
	}

	void update_lru(g_vector<uint32_t>& counters, int used_way) {
        // 时间戳法更新LRU 
    	for (auto& cnt : counters) ++cnt;
        counters[used_way] = 0;
    }

	int find_lru_victim(const g_vector<uint32_t>& counters) {
        // 手动查找最大值的索引（替代std::max_element）
        uint32_t max_val = counters[0];
        int max_idx = 0;
        for (uint64_t i = 1; i < counters.size(); ++i) {
            if (counters[i] > max_val) {
                max_val = counters[i];
                max_idx = i;
            }
        }
        return max_idx;
    }
};

/**
 * @brief 用于过滤被跳过的恒等映射项，并通过更高效的 SRAM 空间利用方式保存这些信息。
 * 		  IdCache条目：8KB超级块的位图（32个256B块）
 * 		  类似于扇区缓存（sector cache）[MASCOTS'00]
 */
class IdCacheEntry
{
public:
	uint32_t super_tag; // 超级块标签（地址的高位部分）
	uint32_t bitmap;  // 32位位图，每bit对应一个块的身份映射状态
	bool valid;
};

class IdCache
{
public:
	struct IdCacheSet{
		g_vector<IdCacheEntry> ways; // 每个set 16ways
		g_vector<uint32_t> access_time; // FIFO

		IdCacheSet() : ways(16),access_time(16,0){}
	};

	g_vector<IdCacheSet> sets; // 256个
	uint32_t timestamp = 0; 

	IdCache() : sets(256){}

	bool lookup(PhysicalAddr pa, uint32_t& out_bitmap)
	{
		uint32_t super_tag = pa >> 7; // 8KB
		uint32_t set_idx = hash_function(super_tag) & 255;

		IdCacheSet& target_set = sets[set_idx];
		for (int i = 0; i < 16; ++i) {
            if (target_set.ways[i].valid && 
                target_set.ways[i].super_tag == super_tag) {
                out_bitmap = target_set.ways[i].bitmap;
                target_set.access_time[i] = ++timestamp; // 更新访问时间
                return true;
            }
        }
		return false;
	}

	void insert(PhysicalAddr pa, uint32_t bitmap)
	{
		uint32_t super_tag = pa >> 7; //8KB
		uint32_t set_idx = hash_function(super_tag) % 256;

		IdCacheSet& target_set = sets[set_idx];
		int victim_way = find_fifo_victim(target_set.access_time);
		target_set.ways[victim_way] = {super_tag, bitmap, true};
		target_set.access_time[victim_way] = ++timestamp;
	}

	uint32_t hash_function(uint32_t key) {
        key = ((key >> 16) ^ key) * 0x45d9f3b;
        key = ((key >> 16) ^ key) * 0x45d9f3b;
        return key >> 16;
    }

	int find_fifo_victim(const g_vector<uint32_t>& access_time) 
	{
		if (access_time.empty()) return -1;
		
		uint32_t min_val = access_time[0];
		int min_idx = 0;
		
		for (uint64_t i = 1; i < access_time.size(); ++i) {
			if (access_time[i] < min_val) {
				min_val = access_time[i];
				min_idx = i;
			}
		}
		return min_idx;
	}
};



/**
 * @brief 叶子节点，每个叶子节点挂4个way
 * @author Jiahao Lu @ XMU
 * @attention 文章还没投出去，这段代码不可以开源
 */
class SDLNode
{
private:
	int last_way = 0;
public:
	// store in 3D-Stacked SRAM
	g_vector<bool> empty_array; // 指示4个way的empty情况
	g_vector<uint64_t> c_tag; // 指示4个压缩后的tag
	g_vector<uint32_t> rrpv_array; // 指示4个way的rrpv值

	// store in DDR
	g_vector<g_vector<bool>> dirty_vector;
	g_vector<g_vector<bool>> valid_vector;


	SDLNode()
	{
		empty_array.resize(4,true);
		c_tag.resize(4,0);
		rrpv_array.resize(4,3); // 按照3来初始化
		dirty_vector.resize(4, g_vector<bool>(64, false)); 
		valid_vector.resize(4, g_vector<bool>(64, false)); 
	}

	/**
	 * @brief 更新RRPVs,保证至少有一个RRPV=3的；被动调用；主动aging待考虑
	 */
	void updRRPV()
	{
		bool hasRRPV = false;
		while(!hasRRPV)
		{
			for(int i = 0 ;i < 4;i++)
			{
				if(rrpv_array[i]==3)
				{
					hasRRPV = true;
					break;
				}
			}
			if(!hasRRPV)
			{
				for(int i = 0 ;i < 4;i++)
				{
					rrpv_array[i] += 1;
				}
			}
		}
	}

	/**
	 * @brief 根据RRPV选择淘汰/驱逐的way
	 * @todo 优化victim_way的选择，拒绝从0开始遍历
	 */
	int findRRPVEvict()
	{
		int victim_way_idx = -1;
		updRRPV(); // 保证有可以替换的
		for(int i = 0; i < 4; i++)
		{
			last_way += 1;
			if(last_way == 4)last_way = 0;
			if(rrpv_array[last_way]==3)
			{
				victim_way_idx = last_way;
				break;
			}
		}
		assert(-1 != victim_way_idx);
		return victim_way_idx;
	}

	/**
	 * @brief 根据way信息更新RRPV的值
	 */
	void resetRRPV(int way, uint32_t set_rrpv = 2)
	{
		rrpv_array[way] = set_rrpv;
	}
};

/**
 * @brief 一个Set对应的SDTree;
 * @author Jiahao Lu @ XMU
 */
class SDTree
{
public:
	bool full_bit; // 指示是否全满
	g_vector<uint16_t> path_bit_array; // 指示path选择 (0上次走右子树，1上次走左子树)
	g_vector<SDLNode> sdnodes; // 叶子节点的集合
	uint16_t tree_height; // 树高（便于树的计算）
	

	/**
	 * @brief 路径的选择；根节点开始检查，根据0/1，指示下一层选择的节点；
	 * 		  当前层为`i`,则左子节点为`2i+1`,右子节点`2i+2`
	 * 		  vector path表示路径包含的节点信息。
	 * 		  返回的是叶子节点索引
	 */
	int path_select()
	{
		int cur_node = 0;
		uint16_t times = tree_height - 1;
		g_vector<int> path;
		path.push_back(cur_node);
		while(times)
		{
			times -= 1;
			if(path_bit_array[cur_node] == 0)
			{
				cur_node = 2 * cur_node + 1;
				path.push_back(cur_node);
			}
			else
			{
				cur_node = 2 * cur_node + 2;
				path.push_back(cur_node);
			}
		}

		// Example node_idx = 3  lnode_idx = 3 + 1 - 2^(2) = 0
		int lnode_idx = path[tree_height-1] + 1 - (int)(pow(2,tree_height-1)+0.5); 
		return lnode_idx;
	}

	/**
	 * @brief 更新是否为满（代码上写需要二重循环，硬件实现的话只需要way/电路并行度个cycles）
	 */
	void updFullState()
	{
		full_bit = 1;
		for(int i = 0; i < (int)(pow(2,tree_height-1)+0.5);i++)
		{
			for(int j = 0; j < 4; j++)
			{
				if(sdnodes[i].empty_array[j]==1)
				{
					full_bit = 0;
					break;
				}
			}
		}
	}

	/**
	 * @brief 返回pair，first表示对应叶子节点索引，second表示叶子节点对应的way的index;
	 *  	  代码实现是二重循环，但是硬件实现的话只需要way/电路并行度个cycles;
	 * 		  只是找到empty way，不会更新full_bit
	 */
	std::pair<int,int> findEmptyWay()
	{
		std::pair<int,int> pij;
		pij = std::make_pair(-1,-1);
		updFullState();
		if(!full_bit)
		{
			for(int i = 0; i < (int)(pow(2,tree_height-1)+0.5);i++)
			{
				for(int j = 0; j < 4;j++)
				{
					if(sdnodes[i].empty_array[j]==true)
					{
						pij = std::make_pair(i,j); 
						break;
					}
				}
			}
		}
		return pij;
	}

	/**
	 * @brief 根据替换/访问叶子节点索引，更新根节点和中间节点的path bit
	 */
	void updNodePathBit(int lnode_idx)
	{
		int rev_idx = (int)(pow(2,tree_height-1)+0.5) - 1 + lnode_idx;
		g_vector<int> rev_path;
		int current_index = rev_idx;
		rev_path.push_back(rev_idx);

		while(current_index > 0)
		{
			int parent_idx = (current_index - 1) / 2;
			rev_path.push_back(parent_idx);
			bool left = current_index == 2 * parent_idx + 1;
			if(left)path_bit_array[parent_idx] = 1; // 本次走左子树，故更新为1
			else path_bit_array[parent_idx] = 0; // 本次走右子树，故更新为0
			current_index = parent_idx;
		}
	};
};



class Way
{
public:
   Address tag;
   bool valid;
   bool dirty;
   uint64_t lru_value;

   g_vector<bool> valid_vector;
   g_vector<bool> dirty_vector;

   void cleanVector()
   {
	  for(uint64_t i = 0; i < valid_vector.size(); i++)
	  {
		valid_vector[i]=false;
		dirty_vector[i]=false;
	  }
   }
};

class Set
{
public:
   Way * ways;
   uint32_t num_ways;

   uint32_t getEmptyWay()
   {
      for (uint32_t i = 0; i < num_ways; i++)
         if (!ways[i].valid)
            return i;
      return num_ways;
   };
   bool hasEmptyWay() { return getEmptyWay() < num_ways; };

   uint32_t findLRUEvictWay()
   {
		if(hasEmptyWay())return getEmptyWay();
		uint64_t max_value = 0;
		uint32_t evict_way_idx = 0;
		for(uint32_t i = 0 ;i < num_ways;i++)
		{
			if(ways[i].lru_value > max_value)
			{
				max_value = ways[i].lru_value;
				evict_way_idx = i;
			}
		}
		return evict_way_idx;
   };

   void updateLRUState(uint32_t way_idx)
   {
		for(uint32_t i=0 ; i < num_ways ;i++)
		{
			if(i != way_idx && ways[i].valid)
			{
				ways[i].lru_value += 1;
			}
		}
		ways[way_idx].lru_value = 0;
		ways[way_idx].valid = true;
   };
};


// Not modeling all details of the tag buffer. 
class TagBufferEntry
{
public:
	Address tag;
	bool remap;
	uint32_t lru;
};

class TagBuffer : public GlobAlloc {
public:
	TagBuffer(Config &config);
	// return: exists in tag buffer or not.
	uint32_t existInTB(Address tag);
	uint32_t getNumWays() { return _num_ways; };

	// return: if the address can be inserted to tag buffer or not.
	bool canInsert(Address tag);
	bool canInsert(Address tag1, Address tag2);
	void insert(Address tag, bool remap);
	double getOccupancy() { return 1.0 * _entry_occupied / _num_ways / _num_sets; };
	void clearTagBuffer();
	void setClearTime(uint64_t time) { _last_clear_time = time; };
	uint64_t getClearTime() { return _last_clear_time; };
private:
	void updateLRU(uint32_t set_num, uint32_t way);
	TagBufferEntry ** _tag_buffer;
	uint32_t _num_ways;
	uint32_t _num_sets;
	uint32_t _entry_occupied;
	uint64_t _last_clear_time;
};

class TLBEntry
{
public:
   uint64_t tag;
   uint64_t way;
   uint64_t count; // for OS based placement policy

   // the following two are only for UnisonCache
   // due to space cosntraint, it is not feasible to keep one bit for each line, 
   // so we use 1 bit for 4 lines.
   uint64_t touch_bitvec; // whether a line is touched in a page
   uint64_t dirty_bitvec; // whether a line is dirty in page
};

class LinePlacementPolicy;
class PagePlacementPolicy;
class OSPlacementPolicy;

//class PlacementPolicy;
class DDRMemory;

class MemoryController : public MemObject {
private:
	DDRMemory * BuildDDRMemory(Config& config, uint32_t frequency, uint32_t domain, g_string name, const std::string& prefix, uint32_t tBL, double timing_scale);
	
	g_string _name;

	// Trace related code
	lock_t _lock;
	lock_t _page_lock;
	bool _collect_trace;
	g_string _trace_dir;
	Address _address_trace[10000];
	uint32_t _type_trace[10000];
	uint32_t _cur_trace_len;
	uint32_t _max_trace_len;

	// External Dram Configuration
	MemObject *	_ext_dram;
	g_string _ext_type; 
public:	
	// MC-Dram Configuration
	MemObject ** _mcdram;
	uint32_t _mcdram_per_mc;
	g_string _mcdram_type;

	uint32_t _fm_size;
	uint32_t _set_assoc;
	g_vector<SDTree> sdtrees;


	// Trimma
	uint32_t block_size; // paper default 256GB
	uint32_t set_assoc; // CLI接收num_ways
	uint32_t irt_levels; // default 3
	NonIdCache nonIdCache;
	IdCache idCache;

	// <set_id,tag,way>
	struct Triplet {
		uint64_t first;
		uint64_t second;
		uint64_t third;
	};
	g_vector<Triplet> _basic_tag_buffer;

	uint64_t getNumRequests() { return _num_requests; };
   	uint64_t getNumSets()     { return _num_sets; };
   	uint32_t getNumWays()     { return _num_ways; };
   	double getRecentMissRate(){ return (double) _num_miss_per_step / (_num_miss_per_step + _num_hit_per_step); };
   	Scheme getScheme()      { return _scheme; };
   	Set * getSets()         { return _cache; };
   	g_unordered_map<Address, TLBEntry> * getTLB() { return &_tlb; };
	TagBuffer * getTagBuffer() { return _tag_buffer; };

	uint64_t getGranularity() { return _granularity; };

private:
	// For Alloy Cache.
	Address transMCAddress(Address mc_addr);
	// For Page Granularity Cache
	Address transMCAddressPage(uint64_t set_num, uint32_t way_num); 

	// For Tagless.
	// For Tagless, we don't use "Set * _cache;" as other schemes. Instead, we use the following 
	// structure to model a fully associative cache with FIFO replacement 
	//vector<Address> _idx_to_address;
	uint64_t _next_evict_idx;
	//map<uint64_t, uint64_t> _address_to_idx;

	// Cache structure
	uint64_t _granularity;
	uint64_t _num_ways;
	uint64_t _cache_size;  // in Bytes
	uint64_t _num_sets;

	int current_tagbuffer_idx;
	bool is_ideal;

	Set * _cache;
	LinePlacementPolicy * _line_placement_policy;
	PagePlacementPolicy * _page_placement_policy;
	OSPlacementPolicy * _os_placement_policy;
	uint64_t _num_requests;
	Scheme _scheme; 
	TagBuffer * _tag_buffer;
	
	// For HybridCache
	uint32_t _footprint_size; 

	// Balance in- and off-package DRAM bandwidth. 
	// From "BATMAN: Maximizing Bandwidth Utilization of Hybrid Memory Systems"
	bool _bw_balance; 
	uint64_t _ds_index;

	// TLB Hack
	g_unordered_map <Address, TLBEntry> _tlb;
	uint64_t _os_quantum;

    // Stats
	Counter _numPlacement;
  	Counter _numCleanEviction;
	Counter _numDirtyEviction;
	Counter _numLoadHit;
	Counter _numLoadMiss;
	Counter _numStoreHit;
	Counter _numStoreMiss;
	Counter _numCounterAccess; // for FBR placement policy  

	Counter _numTagLoad;
	Counter _numTagStore;
	// For HybridCache	
	Counter _numTagBufferFlush;
	Counter _numTBDirtyHit;
	Counter _numTBDirtyMiss;
	// For UnisonCache
	Counter _numTouchedLines;
	Counter _numEvictedLines;

	Counter invalid_data_size;
	Counter valid_data_size;
	Counter migrate_data_size;
	Counter policy_update_size;
	Counter _numTotalHit;
	Counter _numTotalMiss;



	uint64_t _num_hit_per_step;
   	uint64_t _num_miss_per_step;
	uint64_t _mc_bw_per_step;
	uint64_t _ext_bw_per_step;
   	double _miss_rate_trace[MAX_STEPS];

   	uint32_t _num_steps;

	// to model the SRAM tag
	bool 	_sram_tag;
	uint32_t _llc_latency;
public:
	MemoryController(g_string& name, uint32_t frequency, uint32_t domain, Config& config);
	uint64_t access(MemReq& req);
	uint64_t unison_cache_access(MemReq& req);
	uint64_t ideal_unison_access(MemReq& req);
	uint64_t basic_cache_access(MemReq& req);	
	uint64_t theoretical_basic_cache_access(MemReq& req);
	uint64_t test_cache_access(MemReq& req);
	uint64_t ideal_cache_access(MemReq& req);
	uint64_t sdcache_access(MemReq& req);
	uint64_t trimma_access(MemReq& req);
	const char * getName() { return _name.c_str(); };
	void initStats(AggregateStat* parentStat); 
	
	/**
	 * @brief 求log2x
	 */
	uint32_t log2_uint32(uint32_t x) 
	{
		if (x == 0) return 0; 
		return 31 - __builtin_clz(x);
	}
	// Use glob mem
	//using GlobAlloc::operator new;
	//using GlobAlloc::operator delete;
};

#endif