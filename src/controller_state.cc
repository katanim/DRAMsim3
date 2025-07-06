#include "controller_state.h"

namespace dramsim3 {


    ControllerState::ControllerState(int channel_id, const Config &config, int same_row, int same_bank, int same_bank_group, int same_rank) 
                                    : channel_id_(channel_id),  
                                    config_(config),
                                    same_row_(same_row),
                                    same_bank_(same_bank),
                                    same_bank_group_(same_bank_group),
                                    same_rank_(same_rank),
                                    // Initialize the row buffer policy based on the configuration  
                                    row_buf_policy_(config.row_buf_policy == "CLOSE_PAGE"
                                                    ? RowBufPolicy::CLOSE_PAGE
                                                    : RowBufPolicy::OPEN_PAGE) {
        
    }

    ControllerState::~ControllerState() {
        // Destructor logic if needed
    }
    
    //Development idea 2: Keep track of transactions added and completed
    void ControllerState::AddTransaction(uint64_t hex_addr, bool is_write, uint64_t added_cycle) {
        Address addr = config_.AddressMapping(hex_addr);
        auto key = std::make_tuple(addr.rank, addr.bankgroup, addr.bank, addr.row, added_cycle);
        // If there is a transaction in the queue with the same row, bank, bank group, and rank, we can increment the count of same_row_, same_bank_, same_bank_group_, or same_rank_.
        for (const auto& [existing_key, txn] : submitted_transactions) {
            // existing_key is a tuple: (rank, bankgroup, bank, row, added_cycle)
            // Compare with current addr
            if (std::get<0>(existing_key) == addr.rank){
                same_rank_++;
                if (std::get<1>(existing_key) == addr.bankgroup){
                    same_bank_group_++;
                    if (std::get<2>(existing_key) == addr.bank){
                        same_bank_++;
                        if (std::get<3>(existing_key) == addr.row){
                            same_row_++;
                        }
                    }
                } 
                
            } 
        }
        submitted_transactions[key] = {hex_addr, is_write, addr};
        total_transactions_++;
    }
    void ControllerState::RemoveTransaction(uint64_t hex_addr, bool is_write, uint64_t added_cycle) {
        Address addr = config_.AddressMapping(hex_addr);
        auto key = std::make_tuple(addr.rank, addr.bankgroup, addr.bank, addr.row, added_cycle);
        auto it = submitted_transactions.find(key);
        if (it != submitted_transactions.end() && it->second.hex_addr == hex_addr && it->second.is_write == is_write) {
            submitted_transactions.erase(it);
        }
    }
    
    //for open_rows_ buffer. Needed for development idea 1 
    void ControllerState::AddOpenRow(int rank, int bank_group, int bank, int row) {
        auto key = std::make_tuple(rank, bank_group, bank);
        open_rows_[key] = row;
    }

    void ControllerState::RemoveOpenRow(int rank, int bank_group, int bank, int row) {
        auto key = std::make_tuple(rank, bank_group, bank);
        auto it = open_rows_.find(key);
        if (it != open_rows_.end() && it->second == row) {
            //it->first is the key (the tuple of rank, bank_group, bank).
            //it->second is the value (the row number).
            open_rows_.erase(it);
        }
    }

    int ControllerState::OpenRow(int rank, int bank_group, int bank) const {
        auto key = std::make_tuple(rank, bank_group, bank);
        auto it = open_rows_.find(key);
        if (it != open_rows_.end()) {
            return it->second; // return the row number
        }
        return -1; // row not open
    }
}