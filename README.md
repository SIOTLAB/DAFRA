# DAFRA

Dynamic Allocation for Flow-based Redistribution Algorithm.

DAFRA is an Arista EOS agent that reads hardware flow counters, classifies flows based on observed utilization/growth, and updates QoS policers to redistribute unused bandwidth across active flows.

## What It Does

- Polls EOS flow tracking (`tracker t1`) at a fixed interval.
- Maintains per-flow state (rate, EMA rate, growth, policing state).
- Recomputes per-flow policing rates using staged redistribution.
- Programs ACL/class-map/policy-map entries through EOS eAPI.
- Applies `mmf_fair` QoS service policy on configured interfaces.

## Repository Layout

- `main.cpp`: EOS agent lifecycle, timer loop, CLI argument parsing.
- `dafra.h`: data structures, defaults, constants, function declarations.
- `dafra.cpp`: flow parsing, redistribution logic, QoS config application.

## Prerequisites

- Arista EOS SDK (`eosdk`) headers and libraries
- `jansson`
- `libcurl`
- A switch/environment where `show flow tracking sampled flow-table hardware counters tracker t1` is available

## Build

```bash
g++ -g -O0 -std=gnu++14 \
  -I/usr/local/include \
  -L/usr/local/lib \
  -o dafra main.cpp dafra.cpp \
  -leos -ljansson -lcurl -lrt
```

Adjust include/library paths for your EOS SDK installation.

## Run

```bash
./dafra [options]
```

Options:

- `--interval`, `--polling-period <seconds>`: polling interval (default `0.1`)
- `--eta`, `--threshold <value>`: donor threshold eta (default `0.2`)
- `--t`, `--max-timeouts <count>`: number of polling cycles before exit (default `1000`)
- `--gt`, `--tau_g <value>`: growth threshold tau_g (default `0.13`)
- `--et`, `--tau_e <value>`: excess threshold tau_e (default `0.02`)
- `--cb`, `--burst <value>`: burst constant (default `0.25`)
- `-h`, `--help`: print help

Example:

```bash
./dafra --interval 0.1 --eta 0.2 --gt 0.13 --et 0.02 --cb 0.25 --t 2000
```

## Operational Notes

- On startup, the agent performs cleanup of previously created QoS policy/class-map/ACL artifacts and detaches applied QoS service-policy bindings.
- Flow collection, policy attachment targets, and link-capacity assumptions are defined as constants in source code.
- If your topology or interface mapping differs, update those constants before deployment.

## Algorithm Summary

At each epoch:

1. Read current flow counters and update per-flow moving averages.
2. Build a bandwidth pool from flows underutilizing or above fair-share assignments.
3. Satisfy new/smaller flows up to fair share where possible.
4. Redistribute remaining pool across eligible larger flows.
5. Reprogram policers (`police rate ... burst-size ...`) in the `mmf_fair` policy-map.

## Safety / Deployment Guidance

- Validate in a lab before production deployment.
- This agent actively mutates live QoS configuration.
- Use conservative polling/threshold values initially and monitor effect on traffic.

## License

Source headers indicate:

- DAFRA code is released under Apache License 2.0.
- Arista EOS SDK dependency is BSD 3-clause licensed.
