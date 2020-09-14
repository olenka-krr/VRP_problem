#ifndef EVFALG_H
#define EVFALG_H
#include <cstdlib>
#include <cmath>
//#include <cstdio>
#include <iostream>
#include <fstream>
#include <ctime>


extern double Rndm();          // returns uniformly distributed on [0; 1) random number
extern int Vmax(int, int);     // returns max value



class Chromosome{
private:
    unsigned int* permut;      /// permutation of locations (pickups + deliveries)
    unsigned int* indx_permut;  /// index permutation

public:
    static unsigned int N;                               // chromosome_length = 2 * number_of_shipments = 2 * nS

    double cost;

    Chromosome();
    Chromosome(const Chromosome&);                       // конструктор копировани€
   ~Chromosome();


    Chromosome& operator = (const Chromosome&);		     // динамич. присваивание
    Chromosome& Crossover(const Chromosome&, const Chromosome&);
    bool Mutation(double);
    bool UniformDistrRandPermutN();
    int CompareCost(const Chromosome&);

    bool constructSolution();      /// returns the permutation`s cost
};

struct vehicleRoute{
    double startAct;
    double endAct;
    std::string* jobTypes;
    std::string* jobIds;
    double* arrTime;
    double* endTime;     /// = arrTime + 1;
    int currentCapacity;
    double cost;        /// current cost
    int prevLocation;   /// index of previous vehicle`s location, = -1 at the beginning
    int currNumPickups;
    int routePoint;      /// route location index
};

class TriangularDistribution{
    private:
        double h;              // высота >=0 !!!
        double m;              // смещение >=0 !!!
        unsigned int n;        // sample size
        double* sample;        // sample
    public:
        TriangularDistribution(){h = m = 0.0; n = 0; sample = NULL; }
        TriangularDistribution(double, double, unsigned int);
       ~TriangularDistribution();
        unsigned int sampling(double);
};


class Population{
    private:
        unsigned int base;               // base population size
        unsigned int descendants;        // number of descendants
        double h;                        // высота >=0 !!!
        double m;                        // смещение >=0 !!!
        double p_mut;                    // mutation probability >=0 !!!
        Chromosome* popul;               // попул€ци€ (основа + потомки)
        unsigned int* populIndexes;      // индексный массив
        TriangularDistribution* pProb;   // объект с веро€тност€ми треугольного распределени€
    public:
        Population();
        Population(unsigned int, unsigned int, double, double, double);
       ~Population();
        void InitPop();
        void NextGen();
        void setElement0(Chromosome& c) { popul[0] = c; }     // set zero element

        void getElementByIndx(unsigned int i, Chromosome& c) {
            c = popul[populIndexes[i]];
            //c.write();
            //std::cout << std::endl;
        };
        bool ChromInsert(unsigned int, unsigned int);
};


#endif // EVFALG_H
