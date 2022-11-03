/* Author: Constantin Castan, ccastan@uni-muenster.de
 *
 * Implementation of Lifted Multi-agent A* in partial fulfillment
 * of my master's thesis titled
 * "Lifting Multi-agent A* - Implementing a new Optimisation Problem in
 * Isomorphic DecPOMDPs: How many agents do we need?"
 * at the computer science department, University of Münster, 2022.
 * 
 * This file is based on and requires the MADP Toolbox,
 * available at https://github.com/MADPToolbox/MADP.
 */

#include <iostream>
#include "argumentHandlers.h"
#include "argumentUtils.h"
#include "BGIP_SolverCreator_BFS.h"
#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"
#include "GMAA_MAAstar.h"
#include "QMDP.h"

// Program documentation, displayed via --help option
static char doc[] = "Runs Lifted Multi-agent A*";

// Command line option to pass the desired minimum utility
const char *minUtilOption_argp_version = "Minimum desired utility option parser";
static const char *minUtilOption_args_doc = 0;
static const char *minUtilOption_doc =
    "This is the documentation for the minimum desired utility option parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)";

// The group IDs (GID_X) induce an ordering of the child parsers in
// the --help message. See argumentHandlers.cpp in the MADP source
// for reserved IDs.
static const int GID_MU = 4;
static struct argp_option minUtilOption_options[] = {
    {"minUtil", 'o', "FLOAT", 0, "Specifies the minimum utility to achieve"},
    {0}};

float MIN_UTIL;
error_t
minUtilOption_parse_argument(int key, char *arg, struct argp_state *state)
{
    switch (key)
    {
    case 'o':
        MIN_UTIL = atof(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp minUtilOption_argp = {minUtilOption_options, minUtilOption_parse_argument, minUtilOption_args_doc, minUtilOption_doc};
const struct argp_child minUtilOption_child = {&minUtilOption_argp, 0, "Minimum desired utility option", GID_MU};

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    // For GMAA options beyond the standard setting, enable if needed
    // ArgumentHandlers::gmaa_child,
    ArgumentHandlers::qheur_child,
    minUtilOption_child,
    {0}};

// needs to be down here
#include "argumentHandlersPostChild.h"

// Calculates the Lifted MEU of a specific state and obs. history.
double CalculateLiftedUtility(
    const DecPOMDPDiscreteInterface *decpomdp,
    const boost::shared_ptr<JointPolicyDiscretePure> jointPolicy,
    PlanningUnitDecPOMDPDiscrete &pu,
    const Index stateIndex,
    const Index jointOHIndex,
    const unsigned int timestep,
    const unsigned int liftingConstant)
{
    const Index jointActionIndex = jointPolicy->GetJointActionIndex(jointOHIndex);
    const double discount = decpomdp->GetDiscount();
    const double liftedImmediateReward = decpomdp->GetReward(stateIndex, jointActionIndex) * liftingConstant;

    if (timestep >= pu.GetHorizon() - 1)
        return liftedImmediateReward;

    double futureReward = 0.0;

    for (Index nextStateIndex = 0; nextStateIndex < decpomdp->GetNrStates(); ++nextStateIndex)
    {
        const double P_nextState = decpomdp->GetTransitionProbability(stateIndex, jointActionIndex, nextStateIndex);
        const double lifted_P_nextState = pow(P_nextState, liftingConstant);
        double futureExpectedUtility = 0.0;

        for (Index nextJOIndex = 0; nextJOIndex < decpomdp->GetNrJointObservations(); ++nextJOIndex)
        {
            const double P_nextJO = decpomdp->GetObservationProbability(jointActionIndex, nextStateIndex, nextJOIndex);
            const double lifted_P_nextJO = pow(P_nextJO, liftingConstant);
            const Index nextJOHIndex = pu.GetSuccessorJOHI(jointOHIndex, nextJOIndex);
            futureExpectedUtility += lifted_P_nextJO * CalculateLiftedUtility(decpomdp, jointPolicy, pu, nextStateIndex, nextJOHIndex, timestep + 1, liftingConstant);
        }
        futureReward += lifted_P_nextState * futureExpectedUtility;
    }

    return liftedImmediateReward + pow(discount, timestep) * futureReward;
}

// Calculates the Lifted MEU of a policy.
double CalculateLiftedMEU(
    const DecPOMDPDiscreteInterface *decpomdp,
    const boost::shared_ptr<JointPolicyDiscretePure> jointPolicy,
    PlanningUnitDecPOMDPDiscrete &pu,
    const unsigned int liftingConstant)
{
    double v = 0.0;
    for (Index stateIndex = 0; stateIndex < decpomdp->GetNrStates(); ++stateIndex)
    {
        double P_prior = decpomdp->GetInitialStateProbability(stateIndex);
        double utility = CalculateLiftedUtility(decpomdp, jointPolicy, pu, stateIndex, 0, 0, liftingConstant);
        v += P_prior * utility;
    }

    return v;
}

// Main optimization method. Gradually increments agent number and lifting
// constant until the desired lifted MEU is obtained.
unsigned int MinimizeAgentNr(
    const DecPOMDPDiscreteInterface *decpomdp,
    const boost::shared_ptr<JointPolicyDiscretePure> jointPolicy,
    PlanningUnitDecPOMDPDiscrete &pu,
    const double minUtility,
    const int verbosity)
{
    const double meu = pu.GetExpectedReward();
    double liftedMeu = meu;
    unsigned int nrAgents = decpomdp->GetNrAgents();
    const unsigned int nrPartitions = nrAgents; // assuming initially each agent is a representative
    int i = 0;
    while (liftedMeu < minUtility)
    {
        // Every partition contains at least p agents (equally distributed)
        unsigned int p = ++nrAgents / nrPartitions;
        // Some partitions may contain one agent more
        unsigned int q = p + 1;
        // k partitions contain q agents
        unsigned int k = nrAgents % nrPartitions;
        // n partitions contain p agents
        unsigned int n = nrPartitions - k;
        // The liftingConstant is the product over the number of agents per partition
        unsigned int liftingConstant = pow(p, n) * pow(q, k);

        liftedMeu = CalculateLiftedMEU(decpomdp, jointPolicy, pu, liftingConstant);

        if (verbosity >= 1)
            std::cout << "Lifted MEU for " << nrAgents << " agents: " << liftedMeu << std::endl;
    }

    return nrAgents;
}

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse(&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    try
    {
        DecPOMDPDiscreteInterface *decpomdp = ArgumentUtils::GetDecPOMDPDiscreteInterfaceFromArgs(args);
        GeneralizedMAAStarPlannerForDecPOMDPDiscrete *gmaa = 0;
        GeneralizedMAAStarPlannerForDecPOMDPDiscrete *gmaaFirstInstance = 0;
        QFunctionJAOHInterface *q = 0;
        PlanningUnitMADPDiscreteParameters params;
        BGIP_SolverCreatorInterface *bgipsc_p = new BGIP_SolverCreator_BFS<JointPolicyPureVector>(args.verbose, INT_MAX);

        if (bgipsc_p && args.verbose >= 1)
            std::cout << "BGIP_SolverCreatorInterface instance: "
                      << bgipsc_p->SoftPrint()
                      << std::endl;

        // From the MADP authors: We need to make sure that the first GMAA
        // instance exists until the end of the program, as the Q-heuristic
        // will use functionality from it. It's a circular dependence...
        gmaaFirstInstance = new GMAA_MAAstar(bgipsc_p, args.horizon, decpomdp, &params, args.verbose);
        gmaa = gmaaFirstInstance;

        q = new QMDP(gmaa, false);
        q->Compute();
        gmaa->SetQHeuristic(q);
        gmaa->Plan();
        boost::shared_ptr<JointPolicyDiscretePure> foundJPol = gmaa->GetJointPolicyDiscretePure();
        int n = MinimizeAgentNr(decpomdp, foundJPol, *gmaa, MIN_UTIL, args.verbose);
        
        if (args.verbose >= 1) 
            std::cout << foundJPol->SoftPrint() << std::endl;
        
        std::cout
        << n
        << " agents are required to achieve at least the desired utility of "
        << MIN_UTIL
        << std::endl;

        delete bgipsc_p;
        delete q;
        delete gmaa;
        delete decpomdp;
    }
    catch (E &e)
    {
        e.Print();
        exit(-1);
    }
}
