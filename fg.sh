sh run.sh &
perf record -g -p $(sudo lsof -t /home/nightsky/hwc2/LinuxRelease/v2/build/main)
perf script | ./FlameGraph/stackcollapse-perf.pl | ./FlameGraph/flamegraph.pl > flame_graph.svg