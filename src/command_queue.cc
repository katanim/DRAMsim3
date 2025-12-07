#include "command_queue.h"
#include <random>

namespace dramsim3 {

CommandQueue::CommandQueue(int channel_id, const Config& config,
                           const ChannelState& channel_state,
                           SimpleStats& simple_stats)
    : rank_q_empty(config.ranks, true),
      config_(config),
      channel_state_(channel_state),
      simple_stats_(simple_stats),
      is_in_ref_(false),
      queue_size_(static_cast<size_t>(config_.cmd_queue_size)),
      queue_idx_(0),
      clk_(0) {
    if (config_.queue_structure == "PER_BANK") {
        queue_structure_ = QueueStructure::PER_BANK;
        num_queues_ = config_.banks * config_.ranks;
    } else if (config_.queue_structure == "PER_RANK") {
        queue_structure_ = QueueStructure::PER_RANK;
        num_queues_ = config_.ranks;
    } else {
        std::cerr << "Unsupportted queueing structure "
                  << config_.queue_structure << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }

    queues_.reserve(num_queues_);
    for (int i = 0; i < num_queues_; i++) {
        auto cmd_queue = std::vector<Command>();
        cmd_queue.reserve(config_.cmd_queue_size);
        queues_.push_back(cmd_queue);
    }

    //initialize env_state_
    auto init_env = [](FeatureTrack& env) {
        env.current = {0, Command(), 0};
        env.prev = {0, Command(), 0};
        env.Q_values = {};  
        env.history = {};
    };
    init_env(env_state_.num_rw_targeting_cls_bank_over_threshold);
    init_env(env_state_.num_rw_targeting_opn_bank_over_threshold);
    init_env(env_state_.num_rw_row_hits_over_threshold);
    init_env(env_state_.more_reads_than_writes);
    init_env(env_state_.current_cmd_row_hit);
    // init_env(env_state_.write_draining);
}

Command CommandQueue::GetCommandToIssue() {
    for (int i = 0; i < num_queues_; i++) {
        auto& queue = GetNextQueue();
        // if we're refresing, skip the command queues that are involved
        if (is_in_ref_) {
            if (ref_q_indices_.find(queue_idx_) != ref_q_indices_.end()) {
                continue;
            }
        }
        UpdateCurrentState(queue);

        std::vector<Command> ready_cmds = GetAllReadyCommands(queue);
        if (ready_cmds.empty()) {
            continue;
        }

        static thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> explore_dist(0.0f, 1.0f);

        Command cmd;
        if (explore_dist(rng) < exploration_rate_) {
            std::uniform_int_distribution<size_t> idx_dist(0, ready_cmds.size() - 1);
            cmd = ready_cmds[idx_dist(rng)];
        } else {
            cmd = GetHighestQCommand(queue);
        }
        if (cmd.IsValid()) {
            if (cmd.IsReadWrite()) {
                EraseRWCommand(cmd);
            }
            UpdateQValues(cmd);
            return cmd;
        }
    }
    return Command();
}

Command CommandQueue::FinishRefresh() {
    // we can do something fancy here like clearing the R/Ws
    // that already had ACT on the way but by doing that we
    // significantly pushes back the timing for a refresh
    // so we simply implement an ASAP approach
    auto ref = channel_state_.PendingRefCommand();
    if (!is_in_ref_) {
        GetRefQIndices(ref);
        is_in_ref_ = true;
    }

    // either precharge or refresh
    auto cmd = channel_state_.GetReadyCommand(ref, clk_);

    if (cmd.IsRefresh()) {
        ref_q_indices_.clear();
        is_in_ref_ = false;
    }
    return cmd;
}

bool CommandQueue::ArbitratePrecharge(const CMDIterator& cmd_it,
                                      const CMDQueue& queue) const {
    auto cmd = *cmd_it;

    for (auto prev_itr = queue.begin(); prev_itr != cmd_it; prev_itr++) {
        if (prev_itr->Rank() == cmd.Rank() &&
            prev_itr->Bankgroup() == cmd.Bankgroup() &&
            prev_itr->Bank() == cmd.Bank()) {
            return false;
        }
    }

    bool pending_row_hits_exist = false;
    int open_row =
        channel_state_.OpenRow(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
    for (auto pending_itr = cmd_it; pending_itr != queue.end(); pending_itr++) {
        if (pending_itr->Row() == open_row &&
            pending_itr->Bank() == cmd.Bank() &&
            pending_itr->Bankgroup() == cmd.Bankgroup() &&
            pending_itr->Rank() == cmd.Rank()) {
            pending_row_hits_exist = true;
            break;
        }
    }

    bool rowhit_limit_reached =
        channel_state_.RowHitCount(cmd.Rank(), cmd.Bankgroup(), cmd.Bank()) >=
        4;
    if (!pending_row_hits_exist || rowhit_limit_reached) {
        simple_stats_.Increment("num_ondemand_pres");
        return true;
    }
    return false;
}

bool CommandQueue::WillAcceptCommand(int rank, int bankgroup, int bank) const {
    int q_idx = GetQueueIndex(rank, bankgroup, bank);
    return queues_[q_idx].size() < queue_size_;
}

bool CommandQueue::QueueEmpty() const {
    for (const auto &q : queues_) {
        if (!q.empty()) {
            return false;
        }
    }
    return true;
}


bool CommandQueue::AddCommand(Command cmd) {
    auto& queue = GetQueue(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
    if (queue.size() < queue_size_) {
        queue.push_back(cmd);
        rank_q_empty[cmd.Rank()] = false;
        return true;
    } else {
        return false;
    }
}

CMDQueue& CommandQueue::GetNextQueue() {
    queue_idx_++;
    if (queue_idx_ == num_queues_) {
        queue_idx_ = 0;
    }
    return queues_[queue_idx_];
}

void CommandQueue::GetRefQIndices(const Command& ref) {
    if (ref.cmd_type == CommandType::REFRESH) {
        if (queue_structure_ == QueueStructure::PER_BANK) {
            for (int i = 0; i < num_queues_; i++) {
                if (i / config_.banks == ref.Rank()) {
                    ref_q_indices_.insert(i);
                }
            }
        } else {
            ref_q_indices_.insert(ref.Rank());
        }
    } else {  // refb
        int idx = GetQueueIndex(ref.Rank(), ref.Bankgroup(), ref.Bank());
        ref_q_indices_.insert(idx);
    }
    return;
}

int CommandQueue::GetQueueIndex(int rank, int bankgroup, int bank) const {
    if (queue_structure_ == QueueStructure::PER_RANK) {
        return rank;
    } else {
        return rank * config_.banks + bankgroup * config_.banks_per_group +
               bank;
    }
}

CMDQueue& CommandQueue::GetQueue(int rank, int bankgroup, int bank) {
    int index = GetQueueIndex(rank, bankgroup, bank);
    return queues_[index];
}

Command CommandQueue::GetFirstReadyInQueue(CMDQueue& queue) const {
    for (auto cmd_it = queue.begin(); cmd_it != queue.end(); cmd_it++) {
        Command cmd = channel_state_.GetReadyCommand(*cmd_it, clk_);
        if (!cmd.IsValid()) {
            continue;
        }
        if (cmd.cmd_type == CommandType::PRECHARGE) {
            if (!ArbitratePrecharge(cmd_it, queue)) {
                continue;
            }
        } else if (cmd.IsWrite()) {
            if (HasRWDependency(cmd_it, queue)) {
                continue;
            }
        }
        return cmd;
    }
    return Command();
}

void CommandQueue::EraseRWCommand(const Command& cmd) {
    auto& queue = GetQueue(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
    for (auto cmd_it = queue.begin(); cmd_it != queue.end(); cmd_it++) {
        if (cmd.hex_addr == cmd_it->hex_addr && cmd.cmd_type == cmd_it->cmd_type) {
            queue.erase(cmd_it);
            return;
        }
    }
    std::cerr << "cannot find cmd!" << std::endl;
    exit(1);
}

int CommandQueue::QueueUsage() const {
    int usage = 0;
    for (auto i = queues_.begin(); i != queues_.end(); i++) {
        usage += i->size();
    }
    return usage;
}

bool CommandQueue::HasRWDependency(const CMDIterator& cmd_it,
                                   const CMDQueue& queue) const {
    // Read after write has been checked in controller so we only
    // check write after read here
    for (auto it = queue.begin(); it != cmd_it; it++) {
        if (it->IsRead() && it->Row() == cmd_it->Row() &&
            it->Column() == cmd_it->Column() && it->Bank() == cmd_it->Bank() &&
            it->Bankgroup() == cmd_it->Bankgroup()) {
            return true;
        }
    }
    return false;
}

/*Re-inforcement learning*/
// struct QTableKey {
//         int state;
//         int command;
//         int value;
//     };

//     // Keeps track of the current state/value and the history for one feature.
//     struct FeatureTrack {
//         QTableKey current;
//         std::vector<QTableKey> history;
//     };

//  struct EnvState {
//         FeatureTrack num_rw_targeting_cls_bank_over_threshold;
//         FeatureTrack num_rw_targeting_opn_bank_over_threshold;
//         FeatureTrack num_rw_row_hits_over_threshold;
//         FeatureTrack more_reads_than_writes; // 1: more reads, -1: more writes, 0: equal
//         FeatureTrack current_cmd_row_hit; // 1: row hit, 0: row miss
//     };


Command CommandQueue::GetHighestQCommand(CMDQueue& queue) const{
    std::vector<Command> ready_cmds = GetAllReadyCommands(queue);
    if (!ready_cmds.empty()) {
        Command best_cmd = ready_cmds[0];
        float best_q_value = CalculateQValue(best_cmd);
        for (size_t i = 1; i < ready_cmds.size(); i++) {
            float q_value = CalculateQValue(ready_cmds[i]);
            if (q_value > best_q_value) {
                best_q_value = q_value;
                best_cmd = ready_cmds[i];
            }
        }
        return best_cmd;
    }
    return Command();
}

std::vector<Command> CommandQueue::GetAllReadyCommands(CMDQueue& queue) const{
    static int call_count = 0;
    call_count++;
    std::vector<Command> ready_cmds;
    for (auto cmd_it = queue.begin(); cmd_it != queue.end(); cmd_it++) {
        Command cmd = channel_state_.GetReadyCommand(*cmd_it, clk_);
        if (!cmd.IsValid()) {
            continue;
        }
        if (cmd.cmd_type == CommandType::PRECHARGE) {
            if (!ArbitratePrecharge(cmd_it, queue)) {
                continue;
            }
        } else if (cmd.IsWrite()) {
            if (HasRWDependency(cmd_it, queue)) {
                continue;
            }
        }
        ready_cmds.push_back(cmd);
    }
    if (ready_cmds.size() > 1){
        for (const auto& cmd : ready_cmds) {
            std::cout << "GetAllReadyCommands called " << call_count << " times, found " << ready_cmds.size() << " ready commands." << std::endl;
            if (cmd.cmd_type != CommandType::ACTIVATE) {
                std::cout << "Command: " << static_cast<int>(cmd.cmd_type) << std::endl;
            }
        }
    }
    return ready_cmds;
}

float CommandQueue::CalculateQValue(const Command& cmd) const {
    int is_row_hit = 0;
    if (cmd.IsReadWrite()) {
        int open_row =
            channel_state_.OpenRow(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
        if (open_row == cmd.Row()) {
            is_row_hit = 1;
        }
    }

    auto get_q_val = [this, &cmd](const FeatureTrack& env, int state) {
        for (const auto& itr : env.Q_values) {
            if (itr.state == state && itr.cmd.cmd_type == cmd.cmd_type) {
                return itr.value;
            }
        }
        // No existing entry: return a default optimistic value to encourage exploration.
        return 1.0f / (1.0f - discount_factor_);
    };

    float Q_total = 0.0f;
    Q_total += get_q_val(env_state_.num_rw_targeting_cls_bank_over_threshold,
                         env_state_.num_rw_targeting_cls_bank_over_threshold.current.state);
    Q_total += get_q_val(env_state_.num_rw_targeting_opn_bank_over_threshold,
                         env_state_.num_rw_targeting_opn_bank_over_threshold.current.state);
    Q_total += get_q_val(env_state_.num_rw_row_hits_over_threshold,
                         env_state_.num_rw_row_hits_over_threshold.current.state);
    Q_total += get_q_val(env_state_.more_reads_than_writes,
                         env_state_.more_reads_than_writes.current.state);
    Q_total += get_q_val(env_state_.current_cmd_row_hit, is_row_hit);

    return Q_total;
}
void CommandQueue::UpdateQValues(const Command& cmd) {
    float reward = 0;
    // Get reward based on command type
    if (cmd.IsReadWrite()) {
        reward += 1; // additional reward for read/write command
        int open_row =
                channel_state_.OpenRow(cmd.Rank(), cmd.Bankgroup(), cmd.Bank());
        if (open_row == cmd.Row()) {
           env_state_.current_cmd_row_hit.current.state = 1;
        }
    } else {
        env_state_.current_cmd_row_hit.current.state = 0;
    }
    
    // Update Q-values for each feature
    auto set_current = [this, &cmd, reward](FeatureTrack& env) {
        float q_current = GetCurrentQValue(cmd, env);
        env.current = {env.current.state, cmd, q_current};
        env.history.push_back(env.current);

        float q_prev_updated = SarsaUpdate(env.prev.value, reward, q_current);

        for (auto& itr : env.Q_values) {
            if (itr.state == env.prev.state &&
                itr.cmd.cmd_type == env.prev.cmd.cmd_type) {
                itr.value = q_prev_updated;
                env.prev = env.current;
                return;
            }
        }

        // If no existing entry, initialize with updated value and track it.
        env.Q_values.push_back({env.prev.state, env.prev.cmd, q_prev_updated});
        env.prev = env.current;
    };
    set_current(env_state_.num_rw_targeting_cls_bank_over_threshold);
    set_current(env_state_.num_rw_targeting_opn_bank_over_threshold);
    set_current(env_state_.num_rw_row_hits_over_threshold);
    set_current(env_state_.more_reads_than_writes);
    set_current(env_state_.current_cmd_row_hit);
    return;
}
void CommandQueue::UpdateCurrentState(CMDQueue& queue){
    int num_rd = 0;
    int num_wr = 0;
    int row_hits = 0;
    int num_cmds_targeting_cls_banks = 0;
    int num_cmds_targeting_opn_banks = 0;
    for (auto cmd_it = queue.begin(); cmd_it != queue.end(); cmd_it++){
        if (cmd_it->IsRead()) {
            num_rd++;
            int open_row =
                channel_state_.OpenRow(cmd_it->Rank(), cmd_it->Bankgroup(), cmd_it->Bank());
            if (open_row == cmd_it->Row()) {
                row_hits++;
            }
        } else if (cmd_it->IsWrite()) {
            num_wr++;
            int open_row =
                channel_state_.OpenRow(cmd_it->Rank(), cmd_it->Bankgroup(), cmd_it->Bank());
            if (open_row == cmd_it->Row()) {
                row_hits++;
            }
        }
        int open_row =
            channel_state_.OpenRow(cmd_it->Rank(), cmd_it->Bankgroup(), cmd_it->Bank());
        if (open_row != -1) {
            num_cmds_targeting_opn_banks++;
        } else {
            num_cmds_targeting_cls_banks++;
        }

    }
    //update num_rw_targeting_cls_bank_over_threshold
    if (num_cmds_targeting_cls_banks > threshold_) {
        env_state_.num_rw_targeting_cls_bank_over_threshold.current.state = 1;
    } else {
        env_state_.num_rw_targeting_cls_bank_over_threshold.current.state = 0;
    }
    //update num_rw_targeting_opn_bank_over_threshold
    if (num_cmds_targeting_opn_banks > threshold_) {
        env_state_.num_rw_targeting_opn_bank_over_threshold.current.state = 1;
    } else {    
        env_state_.num_rw_targeting_opn_bank_over_threshold.current.state = 0;
    }
    //update num_rw_row_hits_over_threshold
    if (row_hits > threshold_) {
        env_state_.num_rw_row_hits_over_threshold.current.state = 1;
    } else {    
        env_state_.num_rw_row_hits_over_threshold.current.state = 0;
    }
    //update more_reads_than_writes
    if (num_rd > num_wr) {
        env_state_.more_reads_than_writes.current.state = 1;
    } else if (num_wr > num_rd) {
        env_state_.more_reads_than_writes.current.state = -1;
    } else {
        env_state_.more_reads_than_writes.current.state = 0;
    }    
}

void CommandQueue::DumpEnvState(const std::string& filename) const {
    string filename_ = filename + "_history.csv";
    std::ofstream outfile;
    outfile.open(filename_, std::ios_base::app);
    outfile << "env_track,state,cmd,value" << std::endl;
    int history_size = env_state_.num_rw_targeting_cls_bank_over_threshold.history.size();
    for (int i = 0; i < history_size; i++) {
        outfile << "num_rw_targeting_cls_bank_over_threshold,"
                << env_state_.num_rw_targeting_cls_bank_over_threshold.history[i].state << ","
                << env_state_.num_rw_targeting_cls_bank_over_threshold.history[i].cmd.cmd_type  << ","
                << env_state_.num_rw_targeting_cls_bank_over_threshold.history[i].value << std::endl
                << "num_rw_targeting_opn_bank_over_threshold,"
                << env_state_.num_rw_targeting_opn_bank_over_threshold.history[i].state << ","
                << env_state_.num_rw_targeting_opn_bank_over_threshold.history[i].cmd.cmd_type  << ","
                << env_state_.num_rw_targeting_opn_bank_over_threshold.history[i].value <<  std::endl
                << "num_rw_row_hits_over_threshold,"
                << env_state_.num_rw_row_hits_over_threshold.history[i].state << ","
                << env_state_.num_rw_row_hits_over_threshold.history[i].cmd.cmd_type << ","
                << env_state_.num_rw_row_hits_over_threshold.history[i].value << std::endl
                << "more_reads_than_writes,"
                << env_state_.more_reads_than_writes.history[i].state << ","
                << env_state_.more_reads_than_writes.history[i].cmd.cmd_type  << ","
                << env_state_.more_reads_than_writes.history[i].value <<std::endl
                << "current_cmd_row_hit,"
                << env_state_.current_cmd_row_hit.history[i].state<< ","
                << env_state_.current_cmd_row_hit.history[i].cmd.cmd_type << ","
                << env_state_.current_cmd_row_hit.history[i].value << std::endl;

    }
    outfile.close();

    filename_ = filename + "_q_values.csv";
    outfile.open(filename_, std::ios_base::app);
    outfile << "env_track,state,cmd,value" << std::endl;
    

    auto print_q_vals = [&outfile](const FeatureTrack& env, const std::string& env_track) {
        for (const auto& itr : env.Q_values) {
            outfile << env_track << ","
                    << itr.state << ","
                    << itr.cmd.cmd_type  << ","
                    << itr.value << std::endl;
            }
    };
    print_q_vals(env_state_.num_rw_targeting_cls_bank_over_threshold, "num_rw_targeting_cls_bank_over_threshold");
    print_q_vals(env_state_.num_rw_targeting_opn_bank_over_threshold, "num_rw_targeting_opn_bank_over_threshold");
    print_q_vals(env_state_.num_rw_row_hits_over_threshold, "num_rw_row_hits_over_threshold");
    print_q_vals(env_state_.more_reads_than_writes, "more_reads_than_writes");
    print_q_vals(env_state_.current_cmd_row_hit, "current_cmd_row_hit");


}
float CommandQueue::SarsaUpdate(float Q_prev, int reward, float Q_selected) {
    float discounted_future_reward = discount_factor_ * Q_selected;
    float sample_estimate = reward + discounted_future_reward;
    float updated_Q_value = (1.0f - learning_rate_) * Q_prev + learning_rate_ * sample_estimate;
    
    return updated_Q_value;
}

float CommandQueue::GetCurrentQValue(const Command cmd, FeatureTrack& env) const {
    for (const auto& itr : env.Q_values) {
        if (itr.state == env.current.state &&
            itr.cmd.cmd_type == cmd.cmd_type) {
            return itr.value;
        }
    }
    return (1.0f/(1.0f-discount_factor_));
}
}  // namespace dramsim3
