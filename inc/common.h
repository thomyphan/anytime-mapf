#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <map>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <chrono>
#include <utility>
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::map;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
	int location = -1;
	explicit PathEntry(int loc = -1) { location = loc; }
};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);
bool isSamePath(const Path& p1, const Path& p2);

struct IterationStats
{
    int sum_of_costs;
    double runtime;
    int num_of_agents;
    string algorithm;
    int sum_of_costs_lowerbound;
    int num_of_colliding_pairs;
    int success;
    IterationStats(int num_of_agents, int sum_of_costs, double runtime, const string& algorithm,
                   int sum_of_costs_lowerbound = 0, int num_of_colliding_pairs = 0, int success = 0) :
            num_of_agents(num_of_agents), sum_of_costs(sum_of_costs), runtime(runtime),
            sum_of_costs_lowerbound(sum_of_costs_lowerbound), algorithm(algorithm),
            num_of_colliding_pairs(num_of_colliding_pairs), success(success) {}
};

struct PIBTPPS_option{
    int windowSize ;
    bool winPIBTSoft ;
    int timestepLimit ;
};


