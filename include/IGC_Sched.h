#ifndef IGC_SCHED_H
#define IGC_SCHED_H

#include <iostream>
#include <vector>
#include <map>
#include <queue>

#include "Base.h"


static const unsigned SMALL_BLOCK_SIZE = 10;
static const unsigned LARGE_BLOCK_SIZE = 20000;
static const unsigned LARGE_BLOCK_SIZE_RPE = 32000;
static const unsigned PRESSURE_REDUCTION_MIN_BENEFIT = 5; // percentage
static const unsigned PRESSURE_REDUCTION_THRESHOLD = 110;
static const unsigned PRESSURE_HIGH_THRESHOLD = 128;
static const unsigned PRESSURE_REDUCTION_THRESHOLD_SIMD32 = 120;


class SethiUllmanQueue {
// private:
public:
    std::vector<int> MaxRegs;
    std::vector<int> DstSizes;
    DDG* graph;

    SethiUllmanQueue(DDG* graph) : graph(graph) {
        MaxRegs.resize(graph->getNodes().size());
        DstSizes.resize(graph->getNodes().size());
        init();
    }

    //Compute MaxRegs and DstSizes for all nodes in the graph
    void init();

    void calculateSethiUllmanNumber(preNode* node);

    int getSethiUllmanNumber(preNode* node) {
        //TODO: revisar si es necesario
        // if(MaxRegs[node->getID()] - DstSizes[node->getID()] < 0){
        //     return 0;
        // }
        return MaxRegs[node->getID()] - DstSizes[node->getID()];
    }

    int getMaxRegs(preNode* node) {
        return MaxRegs[node->getID()];
    }

    //list scheduling with Sethi-Ullman
    std::vector<preNode*> listScheduling();

    //list scheduling with SU and RP-reduction
    std::vector<preNode*> listSchedulingRP();

    //list scheduling with SU, RP-reduction and Cluster
    std::vector<preNode*> listSchedulingRPCL();
};

class LatencyListScheduler {
private:
    DDG* graph;
public:
    LatencyListScheduler(DDG* graph) : graph(graph) {}

    std::vector<preNode*> listScheduling(int  &cicles);

    std::vector<preNode*> listSchedulingSB(int  &cicles);
};

#endif 