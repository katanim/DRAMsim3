#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H  


#include <vector>
#include <map>
#include <unordered_set>

#include "common.h"
#include "controller.h"
#include "configuration.h"

namespace dramsim3 {

struct SubmittedTransactions {
    uint64_t hex_addr;
    bool is_write;
    Address addr;
};

class ControllerState {
    // class members

    public:
    ControllerState(int channel_id, const Config &config, int same_row = 0, int same_bank = 0, int same_bank_group = 0, int same_rank = 0);

    //settings
    void SetChannelId(int channel_id) { channel_id_ = channel_id; };
    int GetChannelId() const { return channel_id_; };
    const Config &GetConfig() const { return config_; };


    void AddTransaction(uint64_t hex_addr, bool is_writem, uint64_t added_cycle);
    void RemoveTransaction(uint64_t hex_addr, bool is_write, uint64_t added_cycle);
    void printStats() const {
        std::cout << "Total transactions: " << total_transactions_ << std::endl;
        std::cout << "Same row transactions: " << same_row_ << std::endl;
        std::cout << "Same bank transactions: " << same_bank_ << std::endl;
        std::cout << "Same bank group transactions: " << same_bank_group_ << std::endl;
        std::cout << "Same rank transactions: " << same_rank_ << std::endl;
    }

    void updateOpenRows(const ChannelState channel_state);
    void AddOpenRow(int rank, int bank_group, int bank, int row);
    void RemoveOpenRow(int rank, int bank_group, int bank, int row);
    int OpenRow(int rank, int bank_group, int bank) const;

    ~ControllerState();

    private:
    // private members
    int channel_id_;
    const Config &config_;
    RowBufPolicy row_buf_policy_;
    // Development idea 2
    // To keep track of conflicting transactions in the queue
    int same_row_; 
    int same_bank_;
    int same_bank_group_;
    int same_rank_;
    int total_transactions_ = 0; // Total number of transactions added
    std::map<std::tuple<int, int, int, int, uint64_t>, SubmittedTransactions> submitted_transactions; // key: (rank, bank_group, bank, row, added cycle), value: transaction details
    
    //for open_rows_ buffer. Needed for development idea 1 
    std::map<std::tuple<int, int, int>, int> open_rows_; // key: (rank, bank_group, bank), value: (row, rank_id)
};

}


#endif  // CONTROLLER_STATE_H   