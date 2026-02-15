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

/**
 * @file main.cpp
 * @brief Flow-based bandwidth redistribution agent
 *
 * This agent monitors flow utilization and redistributes unused bandwidth
 * from underutilizing flows (donors) to overdemanding flows (receivers).
 * The redistribution follows a proportional variant of
 * Max-Min Fair (MMF) algorithm.
 */

#include <eos/acl.h>
#include <eos/agent.h>
#include <eos/class_map.h>
#include <eos/intf.h>
#include <eos/policy_map.h>
#include <eos/eapi.h>
#include <eos/sdk.h>
#include <eos/tracing.h>
#include <eos/timer.h>
#include <eos/panic.h>

#include <jansson.h>
#include <curl/curl.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <csignal>

#include "dafra.h"
#include <iomanip>

//=============================================================================
// Global Variables and External References
//=============================================================================

extern const std::string egressIntf;
std::unordered_map<std::string, egressPortData> flowDBByEgress;

// Algorithm parameters - can be overridden by command line
double eta = DEFAULT_ETA;
double grow_threshold = 0.13;
double excess_threshold = 0.02;
double constant_burst = CONSTANT_BURST;

double polling_interval;
uint64_t EPOCH_INTERVAL = 0;

//=============================================================================
// FlowCulus Agent Class
//=============================================================================

/**
 * @brief Main agent class that handles flow tracking and bandwidth redistribution
 *
 * This class extends EOS SDK handlers to:
 * 1. Monitor link congestion periodically
 * 2. Track individual flow rates and classify as donors/receivers
 * 3. Redistribute bandwidth using Max-Min Fair algorithm
 * 4. Apply policing policies to hardware
 */
class FlowCulus : public eos::agent_handler, public eos::timeout_handler
{
public:
    eos::eapi_mgr *eapiMgr;
    eos::tracer t;

    int max_timeouts_ = MAX_TIMEOUTS; // Default timeout limit
    explicit FlowCulus(eos::sdk &sdk,
                       double polling_interval_param = POLLING_INTERVAL_DEFAULT,
                       int max_timeouts = MAX_TIMEOUTS,
                       double eta_ = DEFAULT_ETA,
                       double grow_threshold_ = DEFAULT_TAU_G,
                       double excess_threshold_ = DEFAULT_TAU_E,
                       double burst_ = CONSTANT_BURST)
        : eos::agent_handler(sdk.get_agent_mgr()),
          eos::timeout_handler(sdk.get_timeout_mgr()),
          t("dafra")
    {
        polling_interval = polling_interval_param;
        ::eta = eta_;
        ::grow_threshold = grow_threshold_;
        ::excess_threshold = excess_threshold_;
        ::constant_burst = burst_;
        max_timeouts_ = max_timeouts;

        eapiMgr = sdk.get_eapi_mgr();
    }

    //=========================================================================
    // EOS SDK Event Handlers
    //=========================================================================

    void on_initialized() override
    {
        eos::exception_handler_is(panic_handler);
        cleanup_on_start(eapiMgr);
        timeout_time_is(eos::now() + polling_interval);
    }

    /**
     * @brief Main polling loop callback
     */
    void on_timeout() override
    {

        if (EPOCH_INTERVAL >= max_timeouts_)
        {
            handle_agent_termination();
            return;
        }

        get_flow_tracking_and_build_db(eapiMgr, EPOCH_INTERVAL);
        timeout_time_is(eos::now() + polling_interval);
        EPOCH_INTERVAL++;
    }

private:
    //=========================================================================
    // Private Helper Methods
    //=========================================================================

    /**
     * @brief Handle agent termination when max timeouts reached
     */
    void handle_agent_termination()
    {
        cleanup_on_start(eapiMgr);
        get_agent_mgr()->exit();
    }

    /**
     * @brief Panic handler for EOS exceptions
     */
    static void panic_handler(eos::error const &exception)
    {
        std::raise(SIGINT);
        exception.raise();
    }

    /**
     * @brief Print agent uptime for debugging
     */
    void print_agent_uptime()
    {
        double uptime = get_agent_mgr()->agent_uptime();
        t.trace0("Agent uptime: %.2f seconds", uptime);
    }
};

//=============================================================================
// Command Line Parsing and Configuration
//=============================================================================

/**
 * @brief Parse command line arguments and validate parameters
 */
struct ProgramConfig
{
    double polling_period = POLLING_INTERVAL_DEFAULT;
    double eta = DEFAULT_ETA;
    int max_timeouts = MAX_TIMEOUTS;
    double grow_threshold = DEFAULT_TAU_G;
    double excess_threshold = DEFAULT_TAU_E;
    double burst = CONSTANT_BURST;

    bool parse_arguments(int argc, char **argv)
    {
        for (int i = 1; i < argc; i++)
        {
            std::string arg = argv[i];

            if (arg == "--interval" || arg == "--polling-period")
            {
                if (!parse_double_param(argv, i, argc, polling_period, "Polling period: missing argument")) return false;
                if (polling_period <= 0)
                {
                    std::cerr << "Polling period must be positive" << std::endl;
                    return false;
                }
            }
            else if (arg == "--t" || arg == "--max-timeouts")
            {
                if (!parse_int_param(argv, i, argc, max_timeouts, "Max timeouts: missing argument")) return false;
                if (max_timeouts <= 0)
                {
                    std::cerr << "Max timeouts must be positive" << std::endl;
                    return false;
                }
            }
            else if (arg == "--eta" || arg == "--threshold")
            {
                if (!parse_double_param(argv, i, argc, eta, "Eta: missing argument"))  return false;
                if (eta <= 0 || eta > 1.0)
                {
                    std::cerr << "Eta must be between 0 (exclusive) and 1 (inclusive)" << std::endl;
                    return false;
                }
            }
            else if (arg == "--gt" || arg == "--tau_g")
            {
                if (!parse_double_param(argv, i, argc, grow_threshold, "Growth threshold must be provided")) return false;
            }
            else if (arg == "--et" || arg == "--tau_e")
            {
                if (!parse_double_param(argv, i, argc, excess_threshold, "Excess threshold must be provided")) return false;

            }
            else if (arg == "--cb" || arg == "--burst")
            {
                if (!parse_double_param(argv, i, argc, burst, "Burst constant: missing argument")) return false;
                if (burst <= 0 || burst > 1.0)
                {
                    std::cerr << "Burst constant must be between 0 (exclusive) and 1 (inclusive)" << std::endl;
                    return false;
                }
            }
            else if (arg == "-h" || arg == "--help")
            {
                print_usage(argv[0]);
                return false;
            }
            else
            {
                std::cerr << "Unknown argument: " << arg << std::endl;
                print_usage(argv[0]);
                return false;
            }
        }
        return true;
    }

private:
    bool parse_double_param(char **argv, int &i, int argc, double &param, const char *error_msg)
    {
        if (i + 1 < argc)
        {
            param = std::stod(argv[++i]);
            return true;
        }
        std::cerr << error_msg << std::endl;
        return false;
    }

    bool parse_int_param(char **argv, int &i, int argc, int &param, const char *error_msg)
    {
        if (i + 1 < argc)
        {
            param = std::stoi(argv[++i]);
            return true;
        }
        return false;
    }

    bool parse_string_param(char **argv, int &i, int argc, std::string &param)
    {
        if (i + 1 < argc)
        {
            param = argv[++i];
            return true;
        }
        return false;
    }

    void print_usage(const char *program_name)
    {
        std::cout << "Usage: " << program_name << " [options]\n"
                  << "\nFlow-based Bandwidth Redistribution Agent\n"
                  << "Implements Max-Min Fair algorithm\n\n"
                  << "Options:\n"
                  << "  --interval, --polling-period <seconds>  Polling interval (default: " << POLLING_INTERVAL_DEFAULT << ")\n"
                  << "  --eta, --threshold <value>              Rate threshold η for donors (default: " << DEFAULT_ETA << ")\n"
                  << "  --t, --max-timeouts <count>             Maximum polling cycles (default: " << MAX_TIMEOUTS << ")\n"
                  << "  --gt, --tau_g <value>                   Growth threshold τ_g (default: " << DEFAULT_TAU_G << ")\n"
                  << "  --et, --tau_e <value>                   Excess threshold τ_e (default: " << DEFAULT_TAU_E << ")\n"
                  << "  --cb, --burst <value>                   Burst constant (default: " << CONSTANT_BURST << ")\n"
                  << "  -h, --help                              Show this help message\n\n"
                  << "Algorithm Parameters:\n"
                  << "  η (eta):   Flows using < (1-η)×policed_rate become donors\n"
                  << "  τ_g:       Growth rate threshold for donor classification\n"
                  << "  τ_e:       Excess utilization threshold for probing\n"
                  << "  Burst:     Burst size constant for police rate calculation\n";
    }
};

/**
 * @brief Main entry point
 *
 * Parses command line arguments, configures the agent, and starts the EOS SDK main loop
 */
int main(int argc, char **argv)
{
    // Parse configuration
    ProgramConfig config;
    if (!config.parse_arguments(argc, argv))
    {
        return (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) ? 0 : 1;
    }
    try
    {
        eos::sdk sdk;
        FlowCulus agent(sdk,
                        config.polling_period,
                        config.max_timeouts,
                        config.eta,
                        config.grow_threshold,
                        config.excess_threshold,
                        config.burst);

        std::cout << "Starting DAFRA ..." << std::endl;
        sdk.main_loop(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return (-1);
    }
    return (0);
}
