#ifndef __COMMAND_QUEUE_H
#define __COMMAND_QUEUE_H

#include <unordered_set>
#include <vector>
#include <map>
#include <tuple>
#include "channel_state.h"
#include "common.h"
#include "configuration.h"
#include "simple_stats.h"

namespace dramsim3 {

using CMDIterator = std::vector<Command>::iterator;
using CMDQueue = std::vector<Command>;
enum class QueueStructure { PER_RANK, PER_BANK, SIZE };

class CommandQueue {
   public:
    CommandQueue(int channel_id, const Config& config,
                 const ChannelState& channel_state, SimpleStats& simple_stats);
    Command GetCommandToIssue();
    Command FinishRefresh();
    void ClockTick() { clk_ += 1; };
    bool WillAcceptCommand(int rank, int bankgroup, int bank) const;
    bool AddCommand(Command cmd);
    bool QueueEmpty() const;
    int QueueUsage() const;
    void DumpEnvState(const std::string& filename) const;
    std::vector<bool> rank_q_empty;
    

   private:
    bool ArbitratePrecharge(const CMDIterator& cmd_it,
                            const CMDQueue& queue) const;
    bool HasRWDependency(const CMDIterator& cmd_it,
                         const CMDQueue& queue) const;
    Command GetFirstReadyInQueue(CMDQueue& queue) const;
    int GetQueueIndex(int rank, int bankgroup, int bank) const;
    CMDQueue& GetQueue(int rank, int bankgroup, int bank);
    CMDQueue& GetNextQueue();
    void GetRefQIndices(const Command& ref);
    void EraseRWCommand(const Command& cmd);
    Command PrepRefCmd(const CMDIterator& it, const Command& ref) const;

    /*Re-inforcement learning*/
    float learning_rate_ = 1; // alpha
    float discount_factor_ = 0.9; // gamma
    float exploration_rate_ = 0.1; // epsilon
    int threshold_ = 2; // threshold for number of commands needing same activate/precharge
    
    struct QTableKey {
        int state;
        Command cmd;
        float value;
    };

    // Keeps track of the current state/value and the history for one feature.
    struct FeatureTrack {
        QTableKey current;
        QTableKey prev;
        std::vector<QTableKey> Q_values;
        std::vector<QTableKey> history;
    };

    struct EnvState {
        FeatureTrack num_rw_targeting_cls_bank_over_threshold;
        FeatureTrack num_rw_targeting_opn_bank_over_threshold;
        FeatureTrack num_rw_row_hits_over_threshold;
        FeatureTrack more_reads_than_writes; // 1: more reads, -1: more writes, 0: equal
        FeatureTrack current_cmd_row_hit; // 1: row hit, 0: row miss
        // FeatureTrack write_draining; // 1: yes, 0: no
    };

    EnvState env_state_;
    Command GetHighestQCommand(CMDQueue& queue) const;
    int CalculateQValue(const Command& cmd) const;
    void UpdateQValues(const Command& cmd);
    void UpdateCurrentState(CMDQueue& queue);
    float SarsaUpdate(float Q_prev, int reward, float Q_selected);
    float GetCurrentQValue(const Command cmd, FeatureTrack& env) const;

    QueueStructure queue_structure_;
    const Config& config_;
    const ChannelState& channel_state_;
    SimpleStats& simple_stats_;

    std::vector<CMDQueue> queues_;

    // Refresh related data structures
    std::unordered_set<int> ref_q_indices_;
    bool is_in_ref_;

    int num_queues_;
    size_t queue_size_;
    int queue_idx_;
    uint64_t clk_;
};

}  // namespace dramsim3
#endif
