/*
 * @file dafra.cpp
 * @brief Flow tracking and bandwidth redistribution implementation
 * 
 * DAFRA - Dynamic Allocation for Flow-based Redistribution Algorithm
 * Copyright (C) 2026 
 * Author: Ananya Gopal, SIOTLAB

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Note: This project uses Arista eosdk, which is licensed under the BSD 3-clause license.
 */

#include "dafra.h"
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <unistd.h>
#include <chrono>
#include <ctime>

double total_reallocation = 0;
double RATE_MIN_BITS = 0.05 * TOTAL_BW_BITS;
uint64_t BURST_MAX = 4 * 1024 * 1024;
uint64_t THETA_POOL = TOTAL_BW_BITS * (1 - MU);

// Ports being tracked
const std::string egressIntf = "Ethernet43/1";
const std::vector<std::string> interfaces_7280 = {"Ethernet41/1", "Ethernet41/2", "Ethernet41/3", "Ethernet41/4"};

// Policy tracking
std::vector<std::string> seqs_added;
static int seqNumOver = 10;

void get_flow_tracking_and_build_db(const eos::eapi_mgr *eapiMgr, uint64_t epoch_interval)
{

    json_error_t error;
    json_t *root, *flow;
    size_t index;
    eos::eapi_response_t response;
    epoch_changes track_changes{};

    double current_eapi_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count();

    /* Query flow table via EAPI -> This retrieves flows for all egress interfaces. */
    response = eapiMgr->run_show_cmd("show flow tracking sampled flow-table hardware counters tracker t1");
    root = json_loads(response.responses()[0].c_str(), 0, &error);

    if (!root) return;

    json_t *flows = json_object_get(
        json_object_get(
            json_object_get(
                json_object_get(
                    json_object_get(root, "trackers"), "t1"),
                "groups"),
            "IPv4"),
        "flows");

    if (!flows || !json_is_array(flows))
    {
        json_decref(root);
        return;
    }

    auto &flowMap = flowDBByEgress[egressIntf].records;

    if (json_array_size(flows) != 0)
    {
        json_array_foreach(flows, index, flow)
        {
            process_single_flow(std::to_string(index).c_str(), flow, current_eapi_time, flowMap, track_changes);
        }
    }

    json_decref(root);
    redistribute_and_apply_polices(eapiMgr, track_changes, epoch_interval);
}

void process_single_flow(const char *flow_key, json_t *flow_data, double current_time,
                         std::unordered_map<std::string, FlowRecord> &flowMap,
                         epoch_changes &track_changes)
{
    FlowRecord rec;
    uint64_t bytecountCurr;

    if (!extract_flow_info_from_json(flow_data, rec))
        return;

    std::string fid = rec.flowID();

    bytecountCurr = json_number_value(json_object_get(flow_data, "bytesReceived"));
    auto cached_flow_it = flowMap.find(fid);
    if (cached_flow_it != flowMap.end())
    {
        update_existing_flow(cached_flow_it->second, rec, fid, egressIntf,
                             bytecountCurr, current_time, track_changes);
    }
    else
    {
        initialize_new_flow(rec, fid, egressIntf, bytecountCurr, current_time, flowMap);
        track_changes.new_flows_added = 1;
    }
    return;
}

bool extract_flow_info_from_json(json_t *flow_data, FlowRecord &rec)
{
    // Extract source and destination information
    json_t *key = json_object_get(flow_data, "key");

    rec.srcPort = json_integer_value(json_object_get(key, "srcPort"));
    rec.dstPort = json_integer_value(json_object_get(key, "dstPort"));
    rec.srcAddr = json_string_value(json_object_get(key, "srcAddr"));
    rec.dstAddr = json_string_value(json_object_get(key, "dstAddr"));

    rec.ingressIntf = json_string_value(json_object_get(flow_data, "ingressIntf"));

    // rec.tcpFlags = json_string_value(json_object_get(detail, "tcpFlags"));
    rec.startTime = json_real_value(json_object_get(flow_data, "startTime"));
    rec.lastPktTime = json_real_value(json_object_get(flow_data, "updateTime"));

    return true;
}

/**
 * @brief Update an existing flow's metrics.
 */
void update_existing_flow(FlowRecord &cached, const FlowRecord &rec, const std::string &fid,
                          const std::string &egr, uint64_t bytecountCurr, double current_time,
                          epoch_changes &track_changes)
{
    uint64_t deltaBytes = 0;
    double time_delta = current_time - cached.last_eapi_time;

    if (time_delta <= 0.0) return;

    if (bytecountCurr > cached.snapshot.currbyteCount)
    {
        cached.isNew = false;
        track_changes.any_byte_updates = 1;
        deltaBytes = bytecountCurr - cached.snapshot.currbyteCount;

        // Update flow state
        cached.snapshot.prevbyteCount = cached.snapshot.currbyteCount;
        cached.snapshot.currbyteCount = bytecountCurr;
        cached.lastPktTime = rec.lastPktTime;

        // Line 18
        cached.CURRENT_RATE_BPS = (static_cast<double>(deltaBytes)) * BYTES_TO_BITS / time_delta;

        if (cached.PREVIOUS_RATE_HAT_BPS == 0.0 && cached.CURRENT_RATE_HAT_BPS == 0.0)
        {
            // First time initialization
            cached.PREVIOUS_RATE_HAT_BPS = cached.CURRENT_RATE_BPS;
            cached.CURRENT_RATE_HAT_BPS = cached.CURRENT_RATE_BPS;
        }
        cached.CURRENT_RATE_HAT_BPS = (1 - BETA_CR) * cached.PREVIOUS_RATE_HAT_BPS + (BETA_CR * cached.CURRENT_RATE_BPS);
        cached.PREVIOUS_RATE_HAT_BPS = cached.CURRENT_RATE_HAT_BPS;
        double excess_that_we_have = std::max(0.0, cached.POLICED_RATE_BPS - cached.CURRENT_RATE_HAT_BPS);

        if (cached.POLICED_RATE_BPS > 0)
        {
            cached.EXCESS_WE_HAVE_PCT = (excess_that_we_have / cached.POLICED_RATE_BPS);
        }
        else
        {
            cached.EXCESS_WE_HAVE_PCT = -1;
        }
        cached.UNDERUTILIZED_RATE_PCT = cached.CURRENT_RATE_HAT_BPS / std::max(RATE_MIN_BITS, static_cast<double>(cached.POLICED_RATE_BPS));

        double first_term = (1 - BETA_GR) * cached.PREVIOUS_GROWTH_RATE_HAT_BPS;
        double second_term = BETA_GR * (cached.CURRENT_RATE_BPS - cached.PREVIOUS_RATE_BPS) / std::max(cached.PREVIOUS_RATE_BPS, RATE_MIN_BITS);
        cached.GROWTH_RATE_HAT_BPS = first_term + second_term;
        cached.PREVIOUS_GROWTH_RATE_HAT_BPS = cached.GROWTH_RATE_HAT_BPS;
        cached.PREVIOUS_RATE_BPS = cached.CURRENT_RATE_BPS;
        cached.PREVIOUS_RATE_HAT_BPS = cached.CURRENT_RATE_HAT_BPS;
        cached.last_eapi_time = current_time;
    }
}

/**
 * @brief Initialize a new flow in the cache
 */
void initialize_new_flow(FlowRecord &rec, const std::string &fid, const std::string &egr,
                         uint64_t bytecountCurr, double current_time,
                         std::unordered_map<std::string, FlowRecord> &flows_cache)
{
    if (flows_cache.empty())
    {
        resetSequenceNumbers();
    }

    // Initialize new flow state
    rec.snapshot.currbyteCount = bytecountCurr;
    rec.snapshot.prevbyteCount = bytecountCurr;
    rec.CURRENT_RATE_BPS = 0.0;

    rec.GROWTH_RATE_HAT_BPS = 0.0;
    rec.CURRENT_RATE_HAT_BPS = 0.0;
    rec.EXCESS_WE_HAVE_PCT = -1;
    rec.POLICED_RATE_BPS = 0.0;
    rec.UNDERUTILIZED_RATE_PCT = 0.0;

    rec.PREVIOUS_POLICED_RATE_BPS = 0.0;
    rec.PREVIOUS_GROWTH_RATE_HAT_BPS = 0.0;
    rec.PREVIOUS_RATE_BPS = 0.0;
    rec.PREVIOUS_RATE_HAT_BPS = 0.0;
    rec.is_policed = false;
    rec.isNew = true;

    rec.rule_index = getNextSequenceNumber();
    rec.last_eapi_time = current_time;

    // Add to cache
    flows_cache[fid] = rec;
}

static inline void perform_stage_two(uint64_t RATE_FAIR_BITS,
                                     std::vector<FlowRecord *> &participating_flows, uint64_t epoch)
{
    auto &flowMap = flowDBByEgress[egressIntf].records;
    for (auto &flow_pair : flowMap)
    {
        auto &flow = flow_pair.second;
        double extra_bps = 0.0; // Initialize to zero
        if (!flow.isNew)
        {
            if (flow.GROWTH_RATE_HAT_BPS != 0.0 && flow.GROWTH_RATE_HAT_BPS < grow_threshold)
            {
                if (flow.UNDERUTILIZED_RATE_PCT < (1.0 - eta))
                {
                    extra_bps = ETA_REDIS * (std::max(0.0, flow.POLICED_RATE_BPS - flow.CURRENT_RATE_HAT_BPS));
                    THETA_POOL += extra_bps;
                    flow.POLICED_RATE_BPS -= extra_bps;
                    flow.PREVIOUS_POLICED_RATE_BPS = flow.POLICED_RATE_BPS;
                    extra_bps = 0;
                }
            }
            if (flow.POLICED_RATE_BPS > RATE_FAIR_BITS)
            {
                flow.PREVIOUS_POLICED_RATE_BPS = flow.POLICED_RATE_BPS;
                extra_bps = (flow.POLICED_RATE_BPS - RATE_FAIR_BITS);
                THETA_POOL += extra_bps;
                flow.POLICED_RATE_BPS = RATE_FAIR_BITS;
                participating_flows.push_back(&flow);
                extra_bps = 0;
            }
        }
    }
}

/**
 * @brief Print detailed flow allocations and theta pool status
 */
void print_stage_debug_info(const std::string &stage_name, uint64_t RATE_FAIR_BITS,
                            const std::vector<FlowRecord *> &participating_flows)
{
    auto &flowMap = flowDBByEgress[egressIntf].records;

    // Sort flows by destination port in ascending order
    std::vector<std::pair<std::string, FlowRecord>> sortedFlows(flowMap.begin(), flowMap.end());
    std::sort(sortedFlows.begin(), sortedFlows.end(),
              [](const auto &a, const auto &b)
              {
                  return a.second.dstPort < b.second.dstPort;
              });

    // Print table header
    std::cout << "\n"
              << std::string(100, '=') << std::endl;
    std::cout << "*** " << stage_name << " DEBUG INFO ***" << std::endl;
    std::cout << std::string(100, '=') << std::endl;

    std::cout << std::left << std::setw(40)
              << "Flow ID"
              << std::setw(15) << "Current(Mbps)"
              << std::setw(15) << "CR hat(Mbps)"
              << std::setw(15) << "fAR (Mbps)"
              << std::setw(15) << "GR hat"
              << std::setw(15) << "Excess"
              << std::setw(15) << "Util"
              << std::setw(12) << "Prev POLICED RATE"
              << std::setw(10) << "  (In List?)" << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    uint64_t total_allocated = 0;
    uint64_t total_previous_allocated = 0;

    for (const auto &flow_pair : sortedFlows)
    {
        const auto &flow = flow_pair.second;
        std::string fid = flow.flowID();

        double current_mbps = flow.CURRENT_RATE_BPS / 1e6;
        double prev_policed_mbps = flow.PREVIOUS_POLICED_RATE_BPS / 1e6;
        double curr_policed_mbps = flow.POLICED_RATE_BPS / 1e6;

        bool in_participants = std::find(participating_flows.begin(), participating_flows.end(), &flow) != participating_flows.end();

        std::cout << std::left << std::setw(40) << fid
                  << std::setw(15) << std::fixed << std::setprecision(2) << current_mbps
                  << std::setw(15) << std::fixed << std::setprecision(2) << (flow.CURRENT_RATE_HAT_BPS / 1e6)
                  << std::setw(15) << std::fixed << std::setprecision(2) << curr_policed_mbps
                  << std::setw(15) << std::fixed << std::setprecision(4) << flow.GROWTH_RATE_HAT_BPS
                  << std::setw(15) << std::fixed << std::setprecision(4) << (flow.EXCESS_WE_HAVE_PCT)
                  << std::setw(15) << std::fixed << std::setprecision(4) << (flow.UNDERUTILIZED_RATE_PCT)
                  << std::setw(12) << prev_policed_mbps
                  << std::setw(10) << (in_participants ? "YES" : "NO") << std::endl;

        total_allocated += flow.POLICED_RATE_BPS;
        total_previous_allocated += flow.PREVIOUS_POLICED_RATE_BPS;
    }

    // Print summary
    std::cout << std::string(100, '-') << std::endl;
    std::cout << "TOTAL CURRENT: " << (total_allocated / 1e6) << " Mbps" << std::endl;
    std::cout << "TOTAL PREVIOUS: " << (total_previous_allocated / 1e6) << " Mbps" << std::endl;
    std::cout << "LINK CAPACITY: " << (TOTAL_BW_BITS / 1e6) << " Mbps" << std::endl;
    std::cout << "OVER/UNDER CAPACITY: " << ((int64_t)(total_allocated - TOTAL_BW_BITS) / 1e6) << " Mbps" << std::endl;
    std::cout << "\nSUMMARY: THETA=" << (THETA_POOL / 1e6) << "Mbps, ALLOCATED=" << (total_allocated / 1e6)
              << "Mbps, CAPACITY=" << (TOTAL_BW_BITS / 1e6) << "Mbps" << std::endl;
    std::cout << std::string(100, '=') << std::endl;
}

// Give new flows a chance to grow.
// Give flows that below their fair share, a chance to grow if they want to.
static inline void
perform_stage_three_satisfy_small_and_new_flows(uint64_t RATE_FAIR_BITS,
                                                std::vector<FlowRecord *> &participating_flows)
{
    auto &flowMap = flowDBByEgress[egressIntf].records;

    // First, give all new flows their fair share
    for (auto &flow_pair : flowMap)
    {
        auto &flow = flow_pair.second;
        if (flow.isNew)
        {
            if (THETA_POOL >= RATE_FAIR_BITS)
            {
                THETA_POOL -= RATE_FAIR_BITS;
                flow.BASELINE_FAIR_BPS = RATE_FAIR_BITS;
                flow.POLICED_RATE_BPS = RATE_FAIR_BITS;
            }
            else if (THETA_POOL > 0)
            {
                flow.BASELINE_FAIR_BPS = THETA_POOL;
                flow.POLICED_RATE_BPS = THETA_POOL;
                THETA_POOL = 0;
            }
            else
            {
                flow.BASELINE_FAIR_BPS = 0;
                flow.POLICED_RATE_BPS = 0;
            }
        }
        else
        {
            if (flow.POLICED_RATE_BPS < RATE_FAIR_BITS)
            {

                if ((flow.EXCESS_WE_HAVE_PCT != -1) && flow.EXCESS_WE_HAVE_PCT < excess_threshold)
                {
                    double TERM_A = ETA_PROBE * flow.CURRENT_RATE_HAT_BPS;
                    double extra_tokens = std::min(static_cast<double>(RATE_FAIR_BITS - flow.POLICED_RATE_BPS), TERM_A);

                    if (extra_tokens > THETA_POOL) {
                        extra_tokens = THETA_POOL;
                    }
                    THETA_POOL -= extra_tokens;

                    flow.PREVIOUS_POLICED_RATE_BPS = flow.POLICED_RATE_BPS;
                    double possible_new_rate = std::min(static_cast<double>(RATE_FAIR_BITS), flow.CURRENT_RATE_HAT_BPS + TERM_A);

                    flow.POLICED_RATE_BPS += extra_tokens;
                    // std::cout << "\t new fAR for this flow:" << flow.POLICED_RATE_BPS << std::endl;
                }
            }
        }
    }
}

static inline void
perform_stage_three_eval_theta_larger_flows(uint64_t RATE_FAIR_BITS,
                                            std::vector<FlowRecord *> &participating_flows)
{
    double PSI_VALUE = 0;
    // Sort participating flows by POLICED_RATE_BPS in descending order
    std::sort(participating_flows.begin(), participating_flows.end(),
              [](const FlowRecord *a, const FlowRecord *b)
              {
                  return a->PREVIOUS_POLICED_RATE_BPS > b->PREVIOUS_POLICED_RATE_BPS;
              });

    if (THETA_POOL > 0)
    {
        // Collect flows to remove to avoid iterator invalidation
        std::vector<FlowRecord *> flows_to_remove;
        int number_of_flows = participating_flows.size();

        for (auto *flow : participating_flows)
        {
            PSI_VALUE = RATE_FAIR_BITS + THETA_POOL / number_of_flows;
            if (flow->PREVIOUS_POLICED_RATE_BPS <= PSI_VALUE)
            {
                if (flow->EXCESS_WE_HAVE_PCT > excess_threshold && flow->GROWTH_RATE_HAT_BPS < grow_threshold)
                {
                    flow->POLICED_RATE_BPS = flow->PREVIOUS_POLICED_RATE_BPS;
                    uint64_t take_back = flow->POLICED_RATE_BPS - RATE_FAIR_BITS;

                    if (take_back > THETA_POOL) {
                        THETA_POOL = 0;
                    } else {
                        THETA_POOL -= take_back;
                    }

                    // Mark flow for removal
                    flows_to_remove.push_back(flow);
                }
            }
        }

        // Now safely remove the marked flows
        for (auto *flow_to_remove : flows_to_remove)
        {
            participating_flows.erase(
                std::remove(participating_flows.begin(), participating_flows.end(), flow_to_remove),
                participating_flows.end());
        }
    }
}

static inline void
perform_stage_three_resdis_theta_larger_flows(uint64_t RATE_FAIR_BITS,
                                              std::vector<FlowRecord *> &participating_flows)
{
    uint64_t sum = 0;
    std::vector<FlowRecord *> flows_to_remove;

    for (auto *flow : participating_flows)
    {
        sum += flow->PREVIOUS_POLICED_RATE_BPS;
    }

    for (auto *flow : participating_flows)
    {
        // Use double arithmetic to prevent integer overflow
        double proportion = (double)flow->PREVIOUS_POLICED_RATE_BPS / (double)sum;
        double additional_allocation = proportion * (double)THETA_POOL;
        flow->POLICED_RATE_BPS = RATE_FAIR_BITS + (uint64_t)additional_allocation;
        flows_to_remove.push_back(flow);
    }

    // Clear participating flows after redistribution
    for (auto* flow_to_remove : flows_to_remove) {
        participating_flows.erase(
            std::remove(participating_flows.begin(), participating_flows.end(), flow_to_remove),
            participating_flows.end()
        );
    }
}

void redistribute_and_apply_polices(const eos::eapi_mgr *eapiMgr, epoch_changes &changes, uint64_t epoch)
{
    /*
     * To track more interfaces, loop through all the 
     * egress interfaces that need to be tracked. 
     */
    auto &flowMap = flowDBByEgress[egressIntf].records;
    if (flowMap.empty()) return;

    uint64_t RATE_FAIR_BITS = TOTAL_BW_BITS * (1 - MU) / std::max(1UL, flowMap.size());

    /*
        Well if there are no changes we don't need to go ahead.
        - this includes new flows being added or NO updates to any counters.
        - So we can't make new decisions.
    */
    if (!changes.any_byte_updates || changes.new_flows_added) return;

    std::vector<FlowRecord *> participating_flows;
    // Stage 2: Identify donors and collect theta pool
    perform_stage_two(RATE_FAIR_BITS, participating_flows, epoch);
    perform_stage_three_satisfy_small_and_new_flows(RATE_FAIR_BITS, participating_flows);
    if (THETA_POOL > 0 && participating_flows.size() > 0)
    {
        perform_stage_three_eval_theta_larger_flows(RATE_FAIR_BITS, participating_flows);
        if (participating_flows.size() != 0)
        {
            perform_stage_three_resdis_theta_larger_flows(RATE_FAIR_BITS, participating_flows);
        }
    }
    if (participating_flows.size() == 0 && THETA_POOL != 0)
    {
        uint64_t allocate_equally = THETA_POOL / flowMap.size();

        for (auto &flow_pair : flowMap)
        {
            auto &flow = flow_pair.second;
            flow.POLICED_RATE_BPS += allocate_equally;
        }
    }
    create_acl_class_map_and_policy(eapiMgr);
    THETA_POOL = 0;
}

/**
 * @brief Get next sequence number for rule indexing
 */
int getNextSequenceNumber()
{
    int current = seqNumOver;
    seqNumOver += 10;
    return current;
}

/**
 * @brief Reset sequence number to start fresh (call at experiment start)
 */
void resetSequenceNumbers()
{
    seqNumOver = 10;
}

void cleanup_on_start(const eos::eapi_mgr *eapiMgr)
{
    // std::cout << "CLEANUP_ON_START: Removing existing QoS configurations..." << std::endl;
    eos::eapi_response_t response;

    // Remove existing policy-map
    try
    {
        std::vector<std::string> commands = {
            "configure",
            "no policy-map type quality-of-service mmf_fair",
            "clear flow tracking sampled flow-table hardware tracker t1",
            "exit"};
        response = eapiMgr->run_config_cmds(commands);
    }
    catch (const std::exception &e)
    {
        std::cout << "Removed existing policy-map mmf_fair" << std::string(e.what()) << std::endl;
    }

    // Remove existing class-maps (cf1-cf99 pattern)
    for (int i = 10; i <= 1200; i = i + 10)
    {
        try
        {
            std::vector<std::string> commands = {
                "configure",
                "no class-map cf" + std::to_string(i),
                "exit"};

            response = eapiMgr->run_config_cmds(commands);
        }
        catch (const std::exception &e)
        {
            std::string error_msg = e.what();
            // std::cout << "Error removing class-map cf" << std::to_string(i) << ": " << error_msg << std::endl;
        }
    }
    // Remove existing ACLs (f1-f99 pattern)
    for (int i = 10; i <= 1200; i = i + 10)
    {
        try
        {
            std::vector<std::string> commands = {
                "configure",
                "no ip access-list f" + std::to_string(i),
                "exit"};

            response = eapiMgr->run_config_cmds(commands);
        }
        catch (const std::exception &e)
        {
            // Silently ignore errors for non-existent ACLs
            // Only log if it's not a "not found" type error
            std::string error_msg = e.what();
            if (error_msg.find("does not exist") == std::string::npos &&
                error_msg.find("not found") == std::string::npos)
            {
            }
        }
    }
    // Remove any applied service policies from interfaces
    for (const auto &interface_name : interfaces_7280)
    {
        try
        {
            std::vector<std::string> commands = {
                "configure",
                "interface " + interface_name,
                "no service-policy type qos input",
                "exit",
                "exit"};

            response = eapiMgr->run_config_cmds(commands);
        }
        catch (const std::exception &e)
        {
            // Silently ignore if no policy was applied to this interface
            std::cout << "Error removing service-policy from " << interface_name << ": " << e.what() << std::endl;
        }
    }

    response = eapiMgr->run_show_cmd("show flow tracking sampled flow-table hardware counters  tracker t1");
    json_error_t error;
    json_t *root = json_loads(response.responses()[0].c_str(), 0, &error);

    if (!root)
    {
        std::cerr << "Failed to parse JSON response from flow tracking command" << std::endl;
        return;
    }

    json_t *flows = json_object_get(
        json_object_get(
            json_object_get(
                json_object_get(
                    json_object_get(root, "trackers"), "t1"),
                "groups"),
            "IPv4"),
        "flows");
    if (!flows || !json_is_array(flows))
    {
        std::cout << " GOOOD NEWS: No flows found in flow tracking response" << std::endl;
        return;
    }
    else
    {
        std::cout << " BAD NEWS:" << json_array_size(flows) << "flows found in flow tracking response" << std::endl;
    }

    json_decref(root);
}

/**
 * @brief Create and apply ACL, class-map, and policy-map
 */
void create_acl_class_map_and_policy(const eos::eapi_mgr *eapiMgr)
{
    std::vector<std::string> commands;
    std::vector<int> seq;
    commands.push_back("enable");
    commands.push_back("configure terminal");

    for (auto &pair : flowDBByEgress)
    {
        auto &egressIntf = pair.first;
        auto &flowMap = pair.second.records;

        // Add relevant ACLs first.
        for (auto &FLOW : flowMap)
        {
            auto &flow = FLOW.second;
            if (flow.isNew) // Create ACL for new flows.
            {
                seq.push_back(flow.rule_index);
            }
        }

        commands.push_back("policy-map type quality-of-service mmf_fair");

        for (auto &FLOW : flowMap)
        {
            auto &flow = FLOW.second;

            uint64_t first_term = std::ceil(constant_burst * flow.POLICED_RATE_BPS * RTT_MAX);
            uint64_t burst_size = std::min(BURST_MAX, first_term) * BITS_TO_BYTES;

            flow.BURST_BYTES = burst_size;

            std::string police_cmd = "police rate " + std::to_string(flow.POLICED_RATE_BPS) + " bps burst-size " + std::to_string(burst_size) + " bytes";
            commands.push_back("class cf" + std::to_string(flow.rule_index));
            commands.push_back(police_cmd);
        }

        for (auto intf : interfaces_7280)
        {
            commands.push_back("interface " + intf);
            commands.push_back("service-policy type qos input mmf_fair");
        }

        auto response = eapiMgr->run_config_cmds(commands);
        if (!response.success())
        {
            return;
        }

        for (auto &FLOW : flowMap)
        {
            auto &flow = FLOW.second;

            if (std::find(seq.begin(), seq.end(), flow.rule_index) != seq.end())
            {
                seqs_added.push_back(std::to_string(flow.rule_index)); // later used for cleanup
                flow.is_policed = true;
                flow.isNew = false;
                flow.PREVIOUS_POLICED_RATE_BPS = flow.POLICED_RATE_BPS;
            }
        }
    }

    THETA_POOL = 0;
}