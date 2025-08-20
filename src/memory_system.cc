#include "memory_system.h"

namespace dramsim3 {
MemorySystem::MemorySystem(const std::string &config_file,
                           const std::string &output_dir,
                           std::function<void(uint64_t)> read_callback,
                           std::function<void(uint64_t)> write_callback)
    : config_(new Config(config_file, output_dir)) {
    // TODO: ideal memory type?
    if (config_->IsHMC()) {
        dram_system_ = new HMCMemorySystem(*config_, output_dir, read_callback,
                                           write_callback);
    } else {
        dram_system_ = new JedecDRAMSystem(*config_, output_dir, read_callback,
                                           write_callback);
    }
}

MemorySystem::~MemorySystem() {
    delete (dram_system_);
    delete (config_);
}

void MemorySystem::ClockTick() { dram_system_->ClockTick(); }

double MemorySystem::GetTCK() const { return config_->tCK; }

int MemorySystem::GetBusBits() const { return config_->bus_width; }

int MemorySystem::GetBurstLength() const { return config_->BL; }

int MemorySystem::GetQueueSize() const { return config_->trans_queue_size; }

void MemorySystem::RegisterCallbacks(
    std::function<void(uint64_t)> read_callback,
    std::function<void(uint64_t)> write_callback) {
    dram_system_->RegisterCallbacks(read_callback, write_callback);
}

bool MemorySystem::WillAcceptTransaction(uint64_t hex_addr,
                                         bool is_write) const {
    bool ok = dram_system_->WillAcceptTransaction(hex_addr, is_write);
    if (!ok && !is_write) {
        // if it's a read transaction, we can try to find a channel that stores the same data and can accept the transaction
        // this works if writecopy is enabled
        std::map <uint64_t , std::vector<int>> wr_cp_channels_ = dram_system_->GetWriteCopyChannels();
        auto it = wr_cp_channels_.find(hex_addr);
        if (it != wr_cp_channels_.end()) {
            std::vector<int> channels = it->second;
            for (int new_channel : channels) {
                uint64_t newaddr = dram_system_->SetChannel(hex_addr, new_channel); 
                ok = dram_system_->WillAcceptTransaction(newaddr, is_write);
                if (ok) {
                    break; // break after finding a channel that accepts the transaction
                }   
            }
        }
    }                                      
    return ok;
}

bool MemorySystem::AddTransaction(uint64_t hex_addr, bool is_write) {
    return dram_system_->AddTransaction(hex_addr, is_write);
}

void MemorySystem::PrintStats() const { dram_system_->PrintStats(); }

void MemorySystem::ResetStats() { dram_system_->ResetStats(); }

MemorySystem* GetMemorySystem(const std::string &config_file, const std::string &output_dir,
                 std::function<void(uint64_t)> read_callback,
                 std::function<void(uint64_t)> write_callback) {
    return new MemorySystem(config_file, output_dir, read_callback, write_callback);
}
}  // namespace dramsim3

// This function can be used by autoconf AC_CHECK_LIB since
// apparently it can't detect C++ functions.
// Basically just an entry in the symbol table
extern "C" {
void libdramsim3_is_present(void) { ; }
}
