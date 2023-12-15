#include "CorridorReasoning.h"
#include "Conflict.h"
#include <memory>
#include "SpaceTimeAStar.h"
#include "SIPP.h"

shared_ptr<Conflict> CorridorReasoning::run(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths, const HLNode& node)
{
	clock_t t = clock();
	auto corridor = findCorridorConflict(conflict, paths, node);
	accumulated_runtime += (double)(clock() - t) / CLOCKS_PER_SEC;
	return corridor;
}

int CorridorReasoning::findCorridor(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths, int endpoints[], int endpoints_time[]) // return the length of the corridor 
{
	if (paths[conflict->a1]->size() <= 1 || paths[conflict->a2]->size() <= 1)
		return 0;
	assert(conflict->constraint1.size() == 1);
	int  agent, loc1, loc2, t;
	constraint_type type;
	tie(agent, loc2, loc1, t, type) = conflict->constraint1.back();
	if (t < 1)
		return 0;
	if (loc1 < 0) // vertex conflcit
	{
		if (search_engines[0]->instance.getDegree(loc2) != 2)
			return 0; // not a corridor 
		loc1 = loc2;
	}
	else // edge conflict
	{
		if (search_engines[0]->instance.getDegree(loc1) != 2 && search_engines[0]->instance.getDegree(loc2) != 2)
			return 0; // not a corridor 	
	}

	endpoints_time[0] = getExitingTime(*paths[conflict->a1], t); ; // the first timestep when agent 1 exits the corridor 
	endpoints_time[1] = getExitingTime(*paths[conflict->a2], t); ; // the first timestep when agent 2 exits the corridor 
	endpoints[0] = paths[conflict->a1]->at(endpoints_time[0]).location; // the exit location for agent 1
	endpoints[1] = paths[conflict->a2]->at(endpoints_time[1]).location; // the exit location for agent 2
	if (endpoints[0] == endpoints[1]) // agents exit the corridor in the same direction
		return 0;
	// count the distance between the two endpoints, and
	// check whether the corridor between the two exit locations traverse the conflict location, 
	// which indicates whether the two agents come in different directions
	int prev = endpoints[0];
	int curr = paths[conflict->a1]->at(endpoints_time[0] - 1).location;
	bool traverseTheConflictingLocation = false;
	int corridor_length = 1;
	while (curr != endpoints[1])
	{
		if (curr == loc2)
			traverseTheConflictingLocation = true;
		auto neighbors = search_engines[0]->instance.getNeighbors(curr);
		if (neighbors.size() == 2) // inside the corridor
		{
			if (neighbors.front() == prev)
			{
				prev = curr;
				curr = neighbors.back();
			}
			else
			{
				assert(neighbors.back() == prev);
				prev = curr;
				curr = neighbors.front();
			}
		}
		else // exit the corridor without hitting endpoint2
		{ // indicating that the two agents mvoe in the same direction
			return 0;
		}
		corridor_length++;
	}

	if (!traverseTheConflictingLocation)
		return 0;
	// When k=2, it might just be a corner cell, which we do not want to recognize as a corridor
	if (corridor_length == 2 &&
		search_engines[0]->instance.getColCoordinate(endpoints[0]) != search_engines[0]->instance.getColCoordinate(endpoints[1]) &&
		search_engines[0]->instance.getRowCoordinate(endpoints[0]) != search_engines[0]->instance.getRowCoordinate(endpoints[1]))
	{
		return 0;
	}
	return corridor_length;
}


shared_ptr<Conflict> CorridorReasoning::findCorridorConflict(const shared_ptr<Conflict>& conflict,
	const vector<Path*>& paths, const HLNode& node)
{
	int a[2] = { conflict->a1, conflict->a2 };
	int  agent, loc1, loc2, timestep;
	constraint_type type;
	tie(agent, loc1, loc2, timestep, type) = conflict->constraint1.back();
	int curr = -1;
	if (search_engines[0]->instance.getDegree(loc1) == 2)
	{
		curr = loc1;
		if (loc2 >= 0)
			timestep--;
	}
	else if (loc2 >= 0 && search_engines[0]->instance.getDegree(loc2) == 2)
		curr = loc2;
	if (curr <= 0)
		return nullptr;

	int t[2];
	for (int i = 0; i < 2; i++)
		t[i] = getEnteringTime(*paths[a[i]], *paths[a[1 - i]], timestep);
	if (t[0] > t[1])
	{
		int temp = t[0]; t[0] = t[1]; t[1] = temp;
		temp = a[0]; a[0] = a[1]; a[1] = temp;
	}
	int u[2];
	for (int i = 0; i < 2; i++)
		u[i] = paths[a[i]]->at(t[i]).location;
	if (u[0] == u[1])
		return nullptr;
	for (int i = 0; i < 2; i++)
	{
		bool found = false;
		for (int time = t[i]; time < (int)paths[a[i]]->size() && !found; time++)
		{
			if (paths[a[i]]->at(time).location == u[1 - i])
				found = true;
		}
		if (!found)
			return nullptr;
	}
	pair<int, int> edge; // one edge in the corridor
	int corridor_length = getCorridorLength(*paths[a[0]], t[0], u[1], edge);
	int t3, t3_, t4, t4_;
	ConstraintTable ct1(initial_constraints[conflict->a1]);
    ct1.insert2CT(node, conflict->a1);
	t3 = search_engines[conflict->a1]->getTravelTime(paths[conflict->a1]->front().location, u[1], ct1, MAX_TIMESTEP);
	ct1.insert2CT(edge.first, edge.second, 0, MAX_TIMESTEP); // block the corridor in both directions
	ct1.insert2CT(edge.second, edge.first, 0, MAX_TIMESTEP);
	t3_ = search_engines[conflict->a1]->getTravelTime(paths[conflict->a1]->front().location, u[1], ct1, t3 + 2 * corridor_length + 1);
	ConstraintTable ct2(initial_constraints[conflict->a2]);
    ct2.insert2CT(node, conflict->a2);
	t4 = search_engines[conflict->a2]->getTravelTime(paths[conflict->a2]->front().location, u[0], ct2, MAX_TIMESTEP);
	ct2.insert2CT(edge.first, edge.second, 0, MAX_TIMESTEP); // block the corridor in both directions
	ct2.insert2CT(edge.second, edge.first, 0, MAX_TIMESTEP);
	t4_ = search_engines[conflict->a2]->getTravelTime(paths[conflict->a2]->front().location, u[0], ct2, t3 + corridor_length + 1);

    if (abs(t3 - t4) <= corridor_length && t3_ > t3 && t4_ > t4)
    {
		int t1 = std::min(t3_ - 1, t4 + corridor_length);
		int t2 = std::min(t4_ - 1, t3 + corridor_length);
        shared_ptr<Conflict> corridor = make_shared<Conflict>();
        corridor->corridorConflict(conflict->a1, conflict->a2, u[1], u[0], t1, t2);
		if (blocked(*paths[corridor->a1], corridor->constraint1.front()) &&
			blocked(*paths[corridor->a2], corridor->constraint2.front()))
			 return corridor;
    }
    return nullptr;
}


int CorridorReasoning::getExitingTime(const std::vector<PathEntry>& path, int t)
{
	if (t >= (int)path.size())
		t = (int)path.size() - 1;
	int loc = path[t].location;
	while (loc != path.back().location &&
		search_engines[0]->instance.getDegree(loc) == 2)
	{
		t++;
		loc = path[t].location;
	}
	return t;
}


int CorridorReasoning::getEnteringTime(const vector<PathEntry>& path, const vector<PathEntry>& path2, int t)
{
	if (t >= (int)path.size())
		t = (int)path.size() - 1;
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.back().location &&
		search_engines[0]->instance.getDegree(loc) == 2)
	{
		t--;
		loc = path[t].location;
	}
	return t;
}



int CorridorReasoning::getCorridorLength(const vector<PathEntry>& path, int t_start, int loc_end, pair<int, int>& edge)
{
	int curr = path[t_start].location;
	int next;
	int prev = -1;
	int length = 0; // distance to the start location
	int t = t_start;
	bool moveForward = true;
	bool updateEdge = false;
	while (curr != loc_end)
	{
		t++;
		next = path[t].location;
		if (next == curr) // wait
			continue;
		else if (next == prev) // turn aournd
			moveForward = !moveForward;
		if (moveForward)
		{
			if (!updateEdge)
			{
				edge = make_pair(curr, next);
				updateEdge = true;
			}
			length++;
		}
		else
			length--;
		prev = curr;
		curr = next;
	}
	return length;
}

bool CorridorReasoning::blocked(const Path& path, const Constraint& constraint)
{
	int a, loc, t1, t2;
	constraint_type type;
	tie(a, loc, t1, t2, type) = constraint;
	assert(type == constraint_type::RANGE);
	for (int t = t1; t < t2; t++)
	{
		if (t >= (int)path.size() && loc == path.back().location)
			return true;
		else if (t >= 0 && path[t].location == loc)
			return true;
	}
	return false;
}