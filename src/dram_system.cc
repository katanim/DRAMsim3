#include "dram_system.h"

#include <assert.h>

namespace dramsim3 {

// alternative way is to assign the id in constructor but this is less
// destructive
int BaseDRAMSystem::total_channels_ = 0;

BaseDRAMSystem::BaseDRAMSystem(Config &config, const std::string &output_dir,
                               std::function<void(uint64_t)> read_callback,
                               std::function<void(uint64_t)> write_callback)
    : read_callback_(read_callback),
      write_callback_(write_callback),
      last_req_clk_(0),
      config_(config),
      timing_(config_),
#ifdef THERMAL
      thermal_calc_(config_),
#endif  // THERMAL
      clk_(0) {
    total_channels_ += config_.channels;

#ifdef ADDR_TRACE
    std::string addr_trace_name = config_.output_prefix + "addr.trace";
    address_trace_.open(addr_trace_name);
#endif
}



int BaseDRAMSystem::GetChannel(uint64_t hex_addr) const {
    hex_addr >>= config_.shift_bits;
    return (hex_addr >> config_.ch_pos) & config_.ch_mask;
}

// Returns a new address with the channel bits set to new_channel
uint64_t BaseDRAMSystem::SetChannel(uint64_t hex_addr, int new_channel) const {
    // This function is used to update the channel id of a transaction
    uint64_t shifted_bits = hex_addr & ((1ULL << config_.shift_bits) - 1); // first shift_bits bits
    // Clear current channel bits
    uint64_t cleared_addr = (hex_addr>>config_.shift_bits) & ~((config_.ch_mask) << config_.ch_pos);
    // Set new channel bits
    uint64_t new_addr = cleared_addr | (((new_channel & config_.ch_mask)) << config_.ch_pos);
    new_addr <<= config_.shift_bits;
    new_addr |= new_addr | shifted_bits;
    return new_addr;
}

void BaseDRAMSystem::PrintEpochStats() {
    // first epoch, print bracket
    if (clk_ - config_.epoch_period == 0) {
        std::ofstream epoch_out(config_.json_epoch_name, std::ofstream::out);
        epoch_out << "[";
    }
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->PrintEpochStats();
        std::ofstream epoch_out(config_.json_epoch_name, std::ofstream::app);
        epoch_out << "," << std::endl;
    }
#ifdef THERMAL
    thermal_calc_.PrintTransPT(clk_);
#endif  // THERMAL
    return;
}

void BaseDRAMSystem::PrintStats() {
    // Finish epoch output, remove slast comma and append ]
    std::ofstream epoch_out(config_.json_epoch_name, std::ios_base::in |
                                                         std::ios_base::out |
                                                         std::ios_base::ate);
    epoch_out.seekp(-2, std::ios_base::cur);
    epoch_out.write("]", 1);
    epoch_out.close();

    std::ofstream json_out(config_.json_stats_name, std::ofstream::out);
    json_out << "{";

    // close it now so that each channel can handle it
    json_out.close();
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->PrintFinalStats();
        if (i != ctrls_.size() - 1) {
            std::ofstream chan_out(config_.json_stats_name, std::ofstream::app);
            chan_out << "," << std::endl;
        }
        controller_states_[i]->printStats();
    }
    json_out.open(config_.json_stats_name, std::ofstream::app);
    json_out << "}";

#ifdef THERMAL
    thermal_calc_.PrintFinalPT(clk_);
#endif  // THERMAL
}

void BaseDRAMSystem::ResetStats() {
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ResetStats();
    }
}

void BaseDRAMSystem::RegisterCallbacks(
    std::function<void(uint64_t)> read_callback,
    std::function<void(uint64_t)> write_callback) {
    // TODO this should be propagated to controllers
    read_callback_ = read_callback;
    write_callback_ = write_callback;
}

JedecDRAMSystem::JedecDRAMSystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback) {
    if (config_.IsHMC()) {
        std::cerr << "Initialized a memory system with an HMC config file!"
                  << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }

    ctrls_.reserve(config_.channels);
    for (auto i = 0; i < config_.channels; i++) {
#ifdef THERMAL
        ctrls_.push_back(new Controller(i, config_, timing_, thermal_calc_));
#else
        ctrls_.push_back(new Controller(i, config_, timing_));
        controller_states_.push_back(new ControllerState(i,config_));
#endif  // THERMAL
    }
}

JedecDRAMSystem::~JedecDRAMSystem() {
    for (auto it = ctrls_.begin(); it != ctrls_.end(); it++) {
        delete (*it);
    }
}

bool JedecDRAMSystem::WillAcceptTransaction(uint64_t hex_addr,
                                            bool is_write) const {
   
    // if read return true if any of the channels in wr_cp_channels_ can accept the transaction
    auto it = wr_cp_channels_.find(hex_addr);
    if (it != wr_cp_channels_.end() && !is_write) {
        for (auto channel : it->second) {
            uint64_t new_addr = SetChannel(hex_addr, channel); // we could change the address here if needed
            if (ctrls_[channel]->WillAcceptTransaction(new_addr, is_write)) {
                return true;
            }
        }       
        
    }

    //if it's write, return true if any of the channels can accept the transaction
    if(is_write){
        for (int i = 0; i < config_.channels; i++) {
            uint64_t new_addr = SetChannel(hex_addr, i);
            if (ctrls_[i]->WillAcceptTransaction(new_addr, is_write)) {
                return true;
            }
        }

    }
    return false; // if none of the channels can accept the transaction return false
    
}

// bool JedecDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
// // Record trace - Record address trace for debugging or other purposes
// #ifdef ADDR_TRACE
//     address_trace_ << std::hex << hex_addr << std::dec << " "
//                    << (is_write ? "WRITE " : "READ ") << clk_ << std::endl;
// #endif

//     int channel = GetChannel(hex_addr); // Determine the channel based on the address, we could change address here. 
//     bool ok = ctrls_[channel]->WillAcceptTransaction(hex_addr, is_write);

//     // assert(ok);
//     if (ok) {
//         Transaction trans = Transaction(hex_addr, is_write);
//         auto pair = ctrls_[channel]->AddTransaction(trans);
//         // Record the transaction in the controller state
//         controller_states_[channel]->AddTransaction(hex_addr, is_write, pair.second);

//         // Get the current open rows in the channel state
//         controller_states_[channel]->updateOpenRows(ctrls_[channel]->ReturnChannelState());
//     }
//     if (ok && is_write){
//         CopyWrite(hex_addr, is_write); // Copy the write transaction to other channels if possible
//     }
//     if (!ok && !is_write) {
//         auto it = wr_cp_channels_.find(hex_addr);
//         if (it != wr_cp_channels_.end()) {
//             // If the transaction is a read and it was previously copied to other channels
//             // try to read it from those channels
//             std::vector<int> channels = it->second;
//             for (int new_channel : channels) {
//                 // If the transaction has already been copied to other channels, try to read it from those channels instead
//                 uint64_t newaddr = SetChannel(hex_addr, new_channel); // we could change the address here if needed
//                 ok = ctrls_[new_channel]->WillAcceptTransaction(newaddr, is_write);
//                 if (ok) {
//                     auto pair = ctrls_[new_channel]->AddTransaction(Transaction(newaddr, is_write));
//                     // Record the transaction in the controller state
//                     controller_states_[new_channel]->AddTransaction(newaddr, is_write, pair.second);

//                     // Get the current open rows in the channel state
//                     controller_states_[new_channel]->updateOpenRows(ctrls_[new_channel]->ReturnChannelState());
//                     break; // break after finding a channel that accepts the transaction
//                 }   
//             }
//         }
//     }

//     last_req_clk_ = clk_;
//     return ok;
// }

bool JedecDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
// Record trace - Record address trace for debugging or other purposes
#ifdef ADDR_TRACE
    address_trace_ << std::hex << hex_addr << std::dec << " "
                   << (is_write ? "WRITE " : "READ ") << clk_ << std::endl;
#endif
    bool ok = false; // Initialize ok to false, will be set to true if the transaction is added successfully
    std::vector<int> selected_channels = SelectChannelsToAdd(hex_addr, is_write);
    
    for (int channel : selected_channels) {
        uint64_t new_addr = SetChannel(hex_addr, channel); // we could change the address here if needed

        Transaction trans = Transaction(new_addr, is_write);
        auto pair = ctrls_[channel]->AddTransaction(trans);
        // Record the transaction in the controller state
        controller_states_[channel]->AddTransaction(hex_addr, is_write, pair.second);

        // Get the current open rows in the channel state
        controller_states_[channel]->updateOpenRows(ctrls_[channel]->ReturnChannelState());

        ok = true; // Set ok to true if at least one transaction is added successfully
        if (is_write){
            //record the channels that the write transaction was copied to
            wr_cp_channels_[hex_addr].push_back(channel);
        }
        
    }
    return ok;
}

std::vector<int> JedecDRAMSystem::SelectChannelsToAdd(uint64_t hex_addr, bool is_write) {
    std::vector<int> selected_channels;
    int channel = GetChannel(hex_addr);
    int num_channels = config_.channels;
    // If the transaction is a write, return the channels it can be copied to
    if (is_write) {
        for (int i = 0; i < num_channels; i++) {
            uint64_t new_addr = SetChannel(hex_addr, i);
            bool can_accept = ctrls_[channel]->WillAcceptTransaction(new_addr, is_write);
            int queue_size = ctrls_[channel]->QueueUsage();
            if (can_accept && queue_size < config_.cmd_queue_size) {
                selected_channels.push_back(i);
            }
        }
                  
    }

    if (!is_write) {
        // If the transaction is a read, check if it was previously copied to other channels
        auto it = wr_cp_channels_.find(hex_addr);
        if (it != wr_cp_channels_.end()) {
            // If the transaction has already been copied to other channels, search those channels for the one with the smallest queue size
            int selected_channel = it->second.at(0);
            int min_queue_size = ctrls_[selected_channel]->QueueUsage();
            for (auto channel : it->second) {
                // Check if the channel can accept the transaction
                uint64_t new_addr = SetChannel(hex_addr, channel);
                bool can_accept = ctrls_[channel]->WillAcceptTransaction(new_addr, is_write);
                int queue_size = ctrls_[channel]->QueueUsage();
                // // Add to selected channels if it can accept and has space in the command queue
                if (can_accept && queue_size < config_.cmd_queue_size && queue_size <= min_queue_size) {
                    selected_channel = channel;
                    min_queue_size = queue_size;
                }
            }
            selected_channels.push_back(selected_channel);
        } else {
            // If the transaction has not been copied to other channels, return the original channel
            selected_channels.push_back(channel);
        }
    }
     if (selected_channels.empty()) {
        // If no channels can accept the transaction, return the original channel
        selected_channels.push_back(channel);
    }
    //if the transaction is a read, return the fastest channel to read from
    return selected_channels;
}

bool JedecDRAMSystem::CopyWrite(uint64_t hex_addr, bool is_write) {
    // This function is used to copy a write transaction to multiple channels
    int channel = GetChannel(hex_addr);
    int num_channels = config_.channels;
    bool ok = false; // Initialize ok to false, will be set to true if the transaction is added successfully

    if (wr_cp_channels_.find(hex_addr) != wr_cp_channels_.end()) {
        // If the transaction has already been copied to other channels, empty the vector to ensure old info is not kept
        wr_cp_channels_[hex_addr].clear();
    }
    //find channels that the transaction can be copied to
    for (int i = 0; i < num_channels; i++) {
        if (i == channel) {
            continue; // skip the original channel
        }
        // if the channel accepts the transaction
        int new_channel = i;
        uint64_t newaddr = SetChannel(hex_addr, new_channel); // we could change the address here if needed
        if (ctrls_[i]->WillAcceptTransaction(newaddr, is_write)) { 
            // Add the transaction to the controller
            auto pair = ctrls_[i]->AddTransaction(Transaction(newaddr, is_write));
            // Record the transaction in the controller state
            controller_states_[channel]->AddTransaction(newaddr, is_write, pair.second);

            // Get the current open rows in the channel state
            controller_states_[channel]->updateOpenRows(ctrls_[channel]->ReturnChannelState());

            // Record the transaction in the write copy channels
            wr_cp_channels_[hex_addr].push_back(new_channel);
            ok = true;
        }
    }            
    return ok; // return true if the transaction was added successfully
}

void JedecDRAMSystem::ClockTick() {
    for (size_t i = 0; i < ctrls_.size(); i++) {
        // look ahead and return earlier
        while (true) {
            // Updated the pair to truple to include the cycle when the transaction was added
            // This is useful for removing the transaction from the controller state
            auto triple = ctrls_[i]->ReturnDoneTrans(clk_);
            if (std::get<0>(triple) == uint64_t(-1)) {
                break; // no more transactions to return
            }
            //remove the transaction from the controller state
            controller_states_[i]->RemoveTransaction(std::get<0>(triple), std::get<1>(triple), std::get<2>(triple));
            controller_states_[i]->updateOpenRows(ctrls_[i]->ReturnChannelState());
            if (std::get<1>(triple)== 1) {
                write_callback_(std::get<0>(triple));
            } else if (std::get<1>(triple)== 0) {
                read_callback_(std::get<0>(triple));
            } else {
                break;
            }
        }
    }
    for (size_t i = 0; i < ctrls_.size(); i++) {
        ctrls_[i]->ClockTick();
    }
    clk_++;

    if (clk_ % config_.epoch_period == 0) {
        PrintEpochStats();
    }
    return;
}

IdealDRAMSystem::IdealDRAMSystem(Config &config, const std::string &output_dir,
                                 std::function<void(uint64_t)> read_callback,
                                 std::function<void(uint64_t)> write_callback)
    : BaseDRAMSystem(config, output_dir, read_callback, write_callback),
      latency_(config_.ideal_memory_latency) {}

IdealDRAMSystem::~IdealDRAMSystem() {}

bool IdealDRAMSystem::AddTransaction(uint64_t hex_addr, bool is_write) {
    auto trans = Transaction(hex_addr, is_write);
    trans.added_cycle = clk_;
    infinite_buffer_q_.push_back(trans);
    return true;
}

void IdealDRAMSystem::ClockTick() {
    for (auto trans_it = infinite_buffer_q_.begin();
         trans_it != infinite_buffer_q_.end();) {
        if (clk_ - trans_it->added_cycle >= static_cast<uint64_t>(latency_)) {
            if (trans_it->is_write) {
                write_callback_(trans_it->addr);
            } else {
                read_callback_(trans_it->addr);
            }
            trans_it = infinite_buffer_q_.erase(trans_it++);
        }
        if (trans_it != infinite_buffer_q_.end()) {
            ++trans_it;
        }
    }

    clk_++;
    return;
}

}  // namespace dramsim3
