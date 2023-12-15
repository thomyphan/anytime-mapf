#pragma once
#include "ReservationTable.h"
#include "Instance.h"
#include "SingleAgentSolver.h"

class CorridorReasoning
{
public:
	// corridor_strategy strategy;
	double accumulated_runtime = 0;

	CorridorReasoning(const vector<SingleAgentSolver*>& search_engines,
		const vector<ConstraintTable>& initial_constraints):
		search_engines(search_engines), initial_constraints(initial_constraints) {}
	
	shared_ptr<Conflict> run(const shared_ptr<Conflict>& conflict,
		const vector<Path*>& paths, const HLNode& node);

private:
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;

	shared_ptr<Conflict> findCorridorConflict(const shared_ptr<Conflict>& conflict,
		const vector<Path*>& paths, const HLNode& node);
	int findCorridor(const shared_ptr<Conflict>& conflict,
		const vector<Path*>& paths, int endpoints[], int endpoints_time[]); // return the length of the corridor 
	int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t);
	int getExitingTime(const std::vector<PathEntry>& path, int t);
	int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);

	bool blocked(const Path& path, const Constraint& constraint);

};


