/*
 * DAFRA - Dynamic Allocation for Flow-based Redistribution Algorithm
 * Copyright 2026 Ananya Gopal, SIOTLAB
 * Author: Ananya Gopal, SIOTLAB
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// dafra.h
#ifndef DAFRA_H
#define DAFRA_H

#include <eos/eapi.h>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>
#include <jansson.h>
#include <curl/curl.h>
#include <eos/tracing.h>
#include <chrono>
#include <memory>

#define MAX_TIMEOUTS 1000
// Link capacity and thresholds
#define TOTAL_BW_BITS 25000000000
#define TOTAL_BW_BYTES (TOTAL_BW_BITS / 8)
#define BYTES_TO_BITS 8
#define BITS_TO_BYTES 0.125

#define POLLING_INTERVAL_DEFAULT 0.1
#define MU 0.06            // $\mu$
#define DEFAULT_ETA 0.2    // $\eta$
#define DEFAULT_TAU_G 0.13 // $\tau_{G}$
#define DEFAULT_TAU_E 0.02 // $\tau_{E}$
#define ETA_REDIS 0.85
#define ETA_PROBE 0.05
#define CONSTANT_BURST 0.25
#define BETA_CR 0.28 // for T = 100ms, and window - 300ms
#define BETA_GR 0.28

#define RTT_MAX 0.0002

extern double eta;
extern double alpha;
extern double beta;
extern double grow_threshold;
extern double excess_threshold;
extern double constant_burst;
extern const std::string egressIntf;
extern const std::vector<std::string> interfaces_7280;

struct epoch_changes
{
    bool new_flows_added = false;
    bool any_byte_updates = false;
};

struct flowCounterSnapshot
{
    uint64_t currbyteCount = 0;
    uint64_t prevbyteCount = 0;
};

struct FlowRecord
{
    int rule_index = -1;

    std::string srcAddr, dstAddr;
    int srcPort, dstPort;
    std::string ingressIntf;
    std::string tcpFlags;
    ushort ipProto;

    bool isNew = true;

    double CURRENT_RATE_BPS;  /* instantaneous rate in bytes/sec */
    double PREVIOUS_RATE_BPS; /* instantaneous rate in bytes/sec */

    double CURRENT_RATE_HAT_BPS;  /* EMA of current rate */
    double PREVIOUS_RATE_HAT_BPS; /* EMA of previous rate */

    double GROWTH_RATE_HAT_BPS = -100;   /* EMA of previous rate */
    double PREVIOUS_GROWTH_RATE_HAT_BPS; /* EMA of previous rate */

    double EXCESS_WE_HAVE_PCT;     // EXCESS_DEMAND_PCT RATIO
    double UNDERUTILIZED_RATE_PCT; // EVERYTHING IN BYTES NOW


    uint64_t BASELINE_FAIR_BPS;    /* baseline fair share */
    uint64_t POLICED_RATE_BPS = 0; // Current policing rate in bytes per second
    uint64_t PREVIOUS_POLICED_RATE_BPS = 0;
    uint64_t BURST_BYTES = 0; // Measured rate in bytes per second

    bool is_policed : 1; // Whether the flow is currently being policed

    flowCounterSnapshot snapshot;

    double lastPktTime;
    double currPktTime;
    double startTime;
    double last_eapi_time = 0.0; // Timestamp of last EAPI request for this flow

    std::string
    flowID() const
    {
        return srcAddr + ":" + dstAddr + ":" + std::to_string(srcPort) + std::to_string(dstPort);
    }

    void print() const
    {
        auto now = std::chrono::system_clock::now();
        double now_s = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
        std::cout << "\nFLOW:" << now_s << ":" << srcAddr << ":" << srcPort << " → " << dstAddr << ":" << dstPort << ""
                  << "  Ingress: " << ingressIntf << ""
                  << "current_rate: " << CURRENT_RATE_BPS / 1e9 << ""
                  << (is_policed ? "Policed" : "Not Policed") << " Police rate: " << POLICED_RATE_BPS / 1e6 << " Mbps" << "\nseq: " << rule_index << "\n";
    }
};

struct egressPortData
{
    /* Store: ACL name, list of associated ingress interfaces */
    // smetaData mdata;

    /* Store: Actual flows
     * Key: flowID
     * Value: FlowRecord
     */
    std::unordered_map<std::string, FlowRecord> records;
};

extern std::unordered_map<std::string, egressPortData> flowDBByEgress;
void get_flow_tracking_and_build_db(const eos::eapi_mgr *eapiMgr, uint64_t epoch_interval);
void create_acl_class_map_and_policy(const eos::eapi_mgr *eapiMgr);
void cleanup_on_start(const eos::eapi_mgr *eapiMgr);
void redistribute_and_apply_polices(const eos::eapi_mgr *eapiMgr, epoch_changes &changes, uint64_t epoch);
void process_single_flow(const char *flow_key, json_t *flow_data, double current_time,
                         std::unordered_map<std::string, FlowRecord> &flowMap,
                         epoch_changes &changes);
bool extract_flow_info_from_json(json_t *flow_data, FlowRecord &rec);
void update_existing_flow(FlowRecord &cached, const FlowRecord &rec, const std::string &fid,
                          const std::string &egr, uint64_t bytecountCurr, double current_time,
                          epoch_changes &changes);
void initialize_new_flow(FlowRecord &rec, const std::string &fid, const std::string &egr,
                         uint64_t bytecountCurr, double current_time,
                         std::unordered_map<std::string, FlowRecord> &flows_cache);
// Utility functions
int getNextSequenceNumber();
void resetSequenceNumbers();
#endif
