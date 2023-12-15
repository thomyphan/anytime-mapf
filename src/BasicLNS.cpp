#include "BasicLNS.h"
BasicLNS::BasicLNS(const Instance& instance, double time_limit, int neighbor_size, int screen, const string & bandit_algorithm_name, int neighborhoodSizes, int numberOfDestroyHeuristics) :
        instance(instance), time_limit(time_limit), neighbor_size(neighbor_size), screen(screen), bandit_algorithm_name(bandit_algorithm_name), ucb1Constant(100), numberOfNeighborhoodSizeCandidates(neighborhoodSizes), numberOfDestroyHeuristics(numberOfDestroyHeuristics)
{
    if(bandit_algorithm_name == "Random")
    {
        bandit_algorithm = RANDOM_BANDIT;
    }
    if(bandit_algorithm_name == "Roulette")
    {
        bandit_algorithm = ROULETTE_WHEEL;
    }
    if(bandit_algorithm_name == "UCB1")
    {
        bandit_algorithm = UCB1;
    }
    if(bandit_algorithm_name == "Thompson")
    {
        bandit_algorithm = THOMPSON_SAMPLING;
    }
}

void BasicLNS::updateDestroyAndNeighborhoodWeights(const double value, const bool condition)
{
    updateDestroyWeights(&heuristicBanditStats, value, condition);
    if(numberOfNeighborhoodSizeCandidates > 0)
    {
        updateDestroyWeights(neighborhoodBanditStats[selected_neighbor], value, condition);
    }
}

/**
 * Updates the bi-level approach with an observed reward.
 */
void BasicLNS::updateDestroyWeights(BanditStats* stats, double value, const bool condition)
{
    if(bandit_algorithm == RANDOM_BANDIT)
    {
        return;
    }
    stats->destroy_counts[stats->banditIndex] += 1;
    if (condition)
    {
        stats->destroy_weights[stats->banditIndex] += value;
        stats->destroy_weights_squared[stats->banditIndex] += value*value;
    }
}

/**
 * Invokes the bi-level approach to select a destroy heuristic and neighborhood size.
 */
void BasicLNS::sampleDestroyHeuristicAndNeighborhoodSize()
{
    const int numberOfArms = heuristicBanditStats.destroy_weights.size();
    selected_neighbor = sampleDestroyHeuristic(&heuristicBanditStats);
    if(numberOfNeighborhoodSizeCandidates > 0)
    {
        neighbor_arm_index = sampleDestroyHeuristic(neighborhoodBanditStats[selected_neighbor]);
        neighbor_size = 1<<(neighbor_arm_index+1);
    }
}

/**
 * Concrete algorithms for multi-armed bandits like roulette wheel selection, UCB1, and Thompson Sampling.
 */
int BasicLNS::sampleDestroyHeuristic(BanditStats* stats)
{
    const int numberOfArms = stats->destroy_weights.size();
    double weightSum = 0;
    double totalCount = 0;
    double r = (double) rand() / RAND_MAX;
    double threshold = stats->destroy_weights[0];
    for (int index = 0; index < numberOfArms; index++) 
    {
        weightSum += stats->destroy_weights[index];
        totalCount += stats->destroy_counts[index];
    }
    if(bandit_algorithm == RANDOM_BANDIT)
    {
        stats->banditIndex = rand()%stats->destroy_weights.size();
    }
    if(bandit_algorithm == ROULETTE_WHEEL)
    {
        double sum = weightSum;
        if (screen >= 2)
        {
            cout << "destroy weights = ";
            for (const auto& h : stats->destroy_weights)
                cout << h / sum << ",";
        }
        stats->banditIndex = 0;
        while (threshold < r * sum)
        {
            stats->banditIndex++;
            threshold += stats->destroy_weights[stats->banditIndex];
        }
    }
    if(bandit_algorithm == UCB1)
    {
        double maxUCB1value = -std::numeric_limits<double>::max();
        for (int index = 0; index < numberOfArms; index++)
        {
            double numberOfRewards = stats->destroy_counts[index];
            double meanReward = stats->destroy_weights[index]/numberOfRewards;
            double currentUCB1value = 0;
            if (abs(numberOfRewards) < 1e-5)
            {
                currentUCB1value = std::numeric_limits<double>::infinity();
            }
            else
            {
                const double explorationTerm = sqrt(2 * log(totalCount) / numberOfArms);
                currentUCB1value = meanReward + ucb1Constant*explorationTerm;
            }
            if(currentUCB1value > maxUCB1value)
            {
                maxUCB1value = currentUCB1value;
                stats->banditIndex = index;
            }
        }
    }
    if(bandit_algorithm == THOMPSON_SAMPLING)
    {
        double maxValue = -std::numeric_limits<double>::max();
        for (int index = 0; index < numberOfArms; index++)
        {
            double currentValue = 0;
            double n = stats->destroy_counts[index];
            double mean = stats->destroy_weights[index]/n;
            double mean_squared = stats->destroy_weights_squared[index]/n;
            double var = mean_squared - mean*mean;
            if(var < 0)
            {
                var = 0;
            }
            if (abs(n) < 1e-5)
            {
                currentValue = std::numeric_limits<double>::infinity();
            }
            else
            {
                const double delta = mean - mu0;
                const double lambda1 = lambda0 + n;
                assert(lambda1 > 0);
                const double mu1 = (lambda0*mu0 + n*mean) / lambda1;
                const double alpha1 = alpha0 + n / 2;
                assert(alpha1 >= 1);
                const double beta1 = beta0 + 0.5*(n*var + (lambda0*n*delta*delta) / lambda1);
                assert(beta1 >= 0);
                std::gamma_distribution<> gd(alpha1, 1/beta1);
                double gammaVariate = gd(generator);
                const double normalizedVariance = 1.0 / (lambda1*gammaVariate);
                const double normalizedMean = mu1;
                std::normal_distribution<> nd{normalizedMean, sqrt(normalizedVariance)};
                currentValue = nd(generator);
            }
            if(currentValue > maxValue)
            {
                maxValue = currentValue;
                stats->banditIndex = index;
            }
        }
    }
    return stats->banditIndex;
}
