#include "EVFalg.h"
#include "VOGtest.h"
#include "json/json.h"
#include "json/json-forwards.h"

using namespace std;

extern vehicle* Vehicles;    /// array of vehicles
extern int nV;               /// number of vehicles
extern shipment* Shipments;  /// array of shipments
extern int nS;               /// number of shipments

extern double** DistanceMatrix;            /// distances between client`s points, ((2*nS) * (2*nS))
extern double** VehicleDistanceMatrix;     /// distances between vehicles and client`s points, (nV * (2 * nS))

extern int* deliveryIndices;  /// array of nS-size for finding the delivery in permutation, which corresponds given pickup

extern unsigned int* nicks;   ///marks dividing shipments for vehicles, array of (nV - 1) size
extern int* donePickups;      /// array of vehicle`s Id, (nS)

vehicleRoute* vehicleRoutes;         /// array of structures for storing solution

void initVehicleRoutes();            /// initializing of array for vehicle routes


/********************************************Chromosome**************************************************/
/********************************************************************************************************/
Chromosome::Chromosome() {
    if (N == 0) return;
    permut = new unsigned int[N];
    indx_permut = new unsigned int[N];

    for(unsigned int j = 0; j < N; j++) {
        permut[j] = j;
        indx_permut[j] = 0;
    }
    cost = 0;
}

Chromosome::Chromosome(const Chromosome& c){
    if (N == 0) return;
    permut = new unsigned int[N];
    indx_permut = new unsigned int[N];
    for(unsigned int j = 0; j < N; j++) {
        permut[j] = c.permut[j];
        indx_permut[j] = c.indx_permut[j];
    };
    cost = c.cost;
}

Chromosome::~Chromosome() {
    if (permut != NULL)  delete[] permut;
    if (indx_permut != NULL ) delete[] indx_permut;
    cost = 0;
};

Chromosome& Chromosome::operator = (const Chromosome& c){
    if (this == &c) return *this;
    cost = c.cost;

    for (unsigned int i = 0; i < N; i++){
        permut[i] = c.permut[i];
        indx_permut[i] = c.indx_permut[i];
    }
    return *this;
}

bool Chromosome::UniformDistrRandPermutN(){
    unsigned int i, j, v;
    /// Перестановка из n элементов
    /// Обратная перестановка (индексы) - для кроссовера
    /// алгоритм Фишера-Йетса генерации случайной перестановки
    for (i = 0; i < N; i++) {
        j = (unsigned int) round(Rndm() * i);   // uniform random number from 0 to i
        if(i != j)  {
            v = permut[i];
            permut[i] = permut[j];
            permut[j] = v;
        }
    }
    /// permutation of locations (pickups + deliveries)
    /// even numbers - pickups, odd - arrivals.
    /// If the arrival in permutation goes before its pickup then exchange their places

    for(unsigned int i = 0; i < N; ++i)
        if(!(permut[i] % 2))                     /// if the number is even
            for(unsigned int j = 0; j < i; ++j)  /// then look for its odd counterpart in previous part of permutation
                if(permut[j] == permut[i] + 1)   /// if found, exchange their places
                    swap(permut[i], permut[j]);

    for(unsigned int i = 0; i < N; ++i)  indx_permut[permut[i]] = i;

    return true;
}

bool Chromosome::constructSolution(){
    /// initializing of arrays for for solution construction
    initVehicleRoutes();

    /// computing cost
    const int Velocity = 1;
    const int UpDownLoad = 1;
    int iS = 0;        /// index for shipments
    int iV = 0;   /// index for vehicles
    int iV_PU = 0;     /// index for the vehicle with done pickups
    int iP = 0;    /// index for permutation
    int iRP;       /// index for route point
    int doneShipments = 0;

    //cout << "\ncompute cost\n\n";
    /***********************************/
    while(doneShipments < nS){
        ///////////////////////////////
        while( !(permut[iP] % 2) ) {               /// there is always pickup (permut[iP] % 2) at the loop beginning

            if(donePickups[permut[iP] / 2] >= 0){   /// searching for pickup to do
                //cout << iP <<" pickup is ready, searching for pickup to do\n";
                if(++iP == N) {
                    iP = 0;
                    //cout << "***********NEXT ROUND: iP = " << iP << endl;
                    //cin.get();  ////?????
                }

                continue;
            }

            while(vehicleRoutes[iV_PU].currNumPickups == nicks[iV_PU]){  /// searching for available vehicle
                if(++iV_PU == nV) {
                    cout << "*****Error: All vehicles did their jobs*****" << endl;
                    return 0;
                }
            }
            iV = iV_PU;

            while( (Shipments[permut[iP] / 2].capacityDemand > vehicleRoutes[iV].currentCapacity) || (vehicleRoutes[iV].currNumPickups == nicks[iV]) ){ /// searching for available vehicle with enough current capacity
               if(++iV == nV) {
                    //cout << "*****There is no vehicles with capacityDemand at the moment*****" << endl;
                    //cin.get();  ////?????
                    break;    //**** while( Shipments.capacityDemand > vehicleRoutes.currentCapacity )
                }
            } //*** end while( Shipments.capacityDemand > vehicleRoutes.currentCapacity )

            if(iV == nV){
                //cout << iP << ": ***There is no vehicles with capacityDemand at the moment***: continue while\n";
                if(++iP == N) {
                    iP = 0;
                        //cout << "****capasity is not enough*******NEXT ROUND: iPermut = " << iP << endl;
                        //cin.get();    ////?????
                }
                continue;   //***  while( !(permut[iP] % 2) )
            }

            iS = permut[iP] / 2;
            iRP = ++vehicleRoutes[iV].routePoint;
            ++vehicleRoutes[iV].currNumPickups;
            vehicleRoutes[iV].jobTypes[iRP] = "pickupShipment";
            vehicleRoutes[iV].jobIds[iRP] = Shipments[iS].id;
            vehicleRoutes[iV].currentCapacity -= Shipments[iS].capacityDemand;

            if(vehicleRoutes[iV].prevLocation < 0){  /// vehicle goes from its base
                vehicleRoutes[iV].endAct += VehicleDistanceMatrix[iV][2 * iS] * Velocity;
                vehicleRoutes[iV].arrTime[iRP] = vehicleRoutes[iV].endAct;
                vehicleRoutes[iV].cost += VehicleDistanceMatrix[iV][2 * iS] * Vehicles[iV].distance_price;
            } else {
                vehicleRoutes[iV].endAct += DistanceMatrix[vehicleRoutes[iV].prevLocation][2 * iS] * Velocity;
                vehicleRoutes[iV].arrTime[iRP] = vehicleRoutes[iV].endAct;
                vehicleRoutes[iV].cost += DistanceMatrix[vehicleRoutes[iV].prevLocation][2 * iS] * Vehicles[iV].distance_price;

            }


            donePickups[iS] = iV;    /// pickup is done by vehicle with id = iV

            vehicleRoutes[iV].endAct += UpDownLoad;    /// up/download time = 1;
            vehicleRoutes[iV].endTime[iRP] = vehicleRoutes[iV].endAct;

            vehicleRoutes[iV].prevLocation = permut[iP];


            if(++iP == N) {
                iP = 0;
                //cout << "***********NEXT ROUND: iPermut = " << iP << endl;
                //cin.get();    ////?????
            }

        }   //******  end while( !(permut[iP] % 2) )

        ///////////////////////////////////////////////////

        while( permut[iP] % 2 ) {                        /// delivery
            if( (donePickups[(permut[iP] - 1) / 2] < 0) || (donePickups[(permut[iP] - 1) / 2] == nV) ) {  /// pickup is not ready or shipments is done
                //cout << iP << ": pickup is not ready or shipments is done\n\n";
                if(++iP == N) {
                    iP = 0;
                    //cout << "******pickup is not ready or shipments is done*****NEXT ROUND: iPermut = " << iP << endl;
                    //cin.get();    ////?????
                }
                continue;
            }
            iS = (permut[iP] - 1) / 2;
            iV = donePickups[iS];

            iRP = ++vehicleRoutes[iV].routePoint;
            vehicleRoutes[iV].jobTypes[iRP] = "deliveryShipment";
            vehicleRoutes[iV].jobIds[iRP] = Shipments[iS].id;
            vehicleRoutes[iV].currentCapacity += Shipments[iS].capacityDemand;

            vehicleRoutes[iV].endAct += DistanceMatrix[vehicleRoutes[iV].prevLocation][2 * iS] * Velocity;
            vehicleRoutes[iV].arrTime[iRP] = vehicleRoutes[iV].endAct;
            vehicleRoutes[iV].cost += DistanceMatrix[vehicleRoutes[iV].prevLocation][2 * iS] * Vehicles[iV].distance_price;

            vehicleRoutes[iV].endAct += UpDownLoad;    /// up/download time = 1;
            vehicleRoutes[iV].endTime[iRP] = vehicleRoutes[iV].endAct;

            vehicleRoutes[iV].prevLocation = permut[iP];


            ++doneShipments;
            donePickups[iS] = nV;      /// shipment is done
            //cout << "**doneSh = " << doneShipments << endl;

            if(++iP == N) {
                iP = 0;
                //cout << "***********NEXT ROUND: iPermut = " << iP << endl;
                //cin.get();    ////?????
            }

        }   //***** end while( permut[iP] % 2 )


    }   //*****  end while(doneShipments < nS)

    /***********************************/

    cost = 0.;   /// Chromosome.cost

    /// returning to the bases
    for(int i = 0; i < nV; ++i){
        if( vehicleRoutes[i].endAct > 0 ){
            vehicleRoutes[i].endAct += VehicleDistanceMatrix[i][vehicleRoutes[i].prevLocation] * Velocity;
            vehicleRoutes[i].cost += VehicleDistanceMatrix[i][vehicleRoutes[i].prevLocation] * Vehicles[i].distance_price;
            cost += vehicleRoutes[i].cost;
        }
        //cout << "cost." << i << " = " << vehicleRoutes[i].cost << endl;
    }

    //cout << "\n***********Chromosome COST: " << cost << endl;
    return 1;
}



int Chromosome::CompareCost(const Chromosome& c){
   /// Сравниние двух хромосом по значению критерия (cost)
   if ( cost < c.cost ) return -1;
   if ( cost > c.cost ) return +1;
   return 0;
};


Chromosome& Chromosome::Crossover(const Chromosome& c1, const Chromosome& c2) {
    unsigned int i, j1, j2;

    Chromosome h1(c1), h2(c2);

    j1 = 0; j2 = 0;

    for (i = 0; i < N; i++) {
        if (j1 < N) {
            while ( h1.permut[j1] == N ) {
                j1 = j1 + 1;
                if (j1 == N) break;
            }
        }
        if ( j2 < N ) {
            while ( h2.permut[j2] == N ) {
                j2 = j2 + 1;
                if ( j2 == N ) break;
            }
        }
        if ( (j1 < N)  &&  (j2 == N) ) {
            permut[i] = h1.permut[j1];
            h1.permut[j1] = N;
        } else if ( (j2 < N) && (j1 == N) ) {
            permut[i] = h2.permut[j2];
            h2.permut[j2] = N;
        } else if ( (j2 < N) && (j1 < N) ) {
            if ( Rndm() < 0.5 ) {
                permut[i] = h1.permut[j1];
                h2.permut[h2.indx_permut[permut[i]]] = N;
                h1.permut[j1] = N;
            } else {
                permut[i] = h2.permut[j2];
                h1.permut[h1.indx_permut[permut[i]]] = N;
                h2.permut[j2] = N;
            }

        }
        indx_permut[permut[i]] = i;
    }
    return *this;
}

bool Chromosome::Mutation(double p0){
    bool fl = false;
    if ( Rndm() < p0 ) {
        unsigned int j1 = (int)round( Rndm() * (N - 1) );
        unsigned int j2 = (int)round( Rndm() * (N - 1) );
        if ( j1 != j2)  {
            unsigned int v = permut[j1];
            permut[j1] = permut[j2];
            permut[j2] = v;
            indx_permut[permut[j1]] = j1;
            indx_permut[permut[j2]] = j2;
            fl = true;
        }
    }
    return fl;
}

/*************************************TriangularDistribution*********************************************/
/********************************************************************************************************/
// строит треугольное распределение с параметрами h и m, заполняет sample[] - выборку объемом n
TriangularDistribution::TriangularDistribution(double xh, double xm, unsigned int xn){
    h = xh;
    m = xm;
    n = xn;
    sample = new double[n];
    unsigned int i;
    unsigned int s = (unsigned int) m;
    double v;

    for(i = 0; i < n; i++) sample[i] = 0;

    double rn, ri;
    rn = n;
    ///sample[0] = 0 !!! always!/**************************!!!!!!!!!**********/
    for(i = 0; i < n; i++) {
        ri = i;
        if (i <= s )  {
            v =  (2. +  h * ri / m) / rn;
            sample[i] = v * ri / (h + 2.);
        } else {
            v = (2. + h + (h / (rn - m)) * (rn - ri)) / rn;
            sample[i] = v * (ri - m) / (h + 2.) + sample[s];
        }
    }
}

TriangularDistribution::~TriangularDistribution(){
    if (sample != NULL) delete[] sample;
    n = 0;
    m = h = 0.0;
}

unsigned int TriangularDistribution::sampling(double p){
    unsigned int i, i1, j1;
    i1 = 0;
    j1 = n;
    i = (unsigned int) (n / 2);
    while (i != i1) {
        if (p < sample[i]) {
            j1 = i;
        } else {
            i1 = i;
        }
        i = (unsigned int)  (i1 + (j1 - i1) / 2);
    }
    return i;
}

/*******************************************Population***************************************************/
/********************************************************************************************************/

Population::Population(){
    base = descendants = 0;
    h = m = p_mut = 0.0;
    popul = NULL;
    populIndexes = NULL;
    pProb = NULL;
}

Population::~Population(){
    if (populIndexes != NULL ) delete[] populIndexes;
    if (pProb != NULL ) delete pProb;
    base = descendants = 0;
    h = m = p_mut = 0.0;
}

Population::Population(unsigned int b, unsigned int d, double hight, double mean, double pMut){
    base = b;
    descendants = d;
    h = hight;
    m = mean;
    p_mut = pMut;
    popul = new Chromosome[base + descendants];

    if ( popul == NULL ) cout << "Failed to allocate memory for Chromosome* popul\n" << endl;

    populIndexes = new unsigned int[base + descendants];
    if ( populIndexes == NULL ) cout << "Failed to allocate memory for unsigned int* populIndexes\n" << endl;

    for ( unsigned int i = 0; i < (base + descendants); i++ ) populIndexes[i] = i;
    // распределение вероятностей при отборе особей для кроссовера
    pProb = new TriangularDistribution(h * base, m * base, base);
}

bool Population::ChromInsert(unsigned int n, unsigned int m){
    // Бинарный поиск
    // Процедура вставки в индексный массив (c учетом порядка).
    // m -й элемент массива ищет свое место  m > n!!!

    if (m < n) return false;

    unsigned int i, j;
    i = populIndexes[n];
    j = populIndexes[m];
    //cout << "n = " << n << " i = " << i << " m = " << m << " j = "  << j << endl;
    /// New element is compared with last element of Population, if it is worse then return
    if ( popul[i].CompareCost(popul[j]) <= 0 ) return true;

    unsigned int k, p = 0, q = n, i0;
    int cmp;

    if (n > 0){
        //cout << "n = " << n << endl;
        while ( q >= p ){
            k = p + (unsigned int) ( ( q - p + 1) / 2 );

            i = populIndexes[k];
            cmp = popul[i].CompareCost(popul[j]);

            if (k == 0){
                if (cmp == 1) i0 = k;
                else i0 = k + 1;
                break;
            }

            if (cmp == 0){
                i0 = k;
                break;
            } else if( cmp == -1 ){
                p = k + 1;
                i0 = k;
            } else {
                q = k - 1;
                i0 = q;
            }
            //cout << "q = " << q << " p = " << p << " k = " << k << " i0 = " << i0 << " cmp = " << cmp << endl;
        }

    } else {  // n = 0
        //cout << "n = " << n << " m = " << m << endl;
        populIndexes[n] = m;        // m = 1
        populIndexes[n + 1] = n;
        //cout << "\n New record: " << m << "\n\n";
        return true;
    }


    i = populIndexes[n];

    //if (i0 == 0) cout << "\n New record: " << j << "\n\n";

    for (k = n; k > i0; k--)  populIndexes[k] = populIndexes[k - 1];

    populIndexes[i0] = j;
    populIndexes[m] = i;

    return true;
}

void Population::InitPop(){
    for(unsigned int i = 1; i < base; i++) {
        popul[i].UniformDistrRandPermutN();
        popul[i].constructSolution();

        // Вставка элемента в индексный массив
        ChromInsert(i - 1, i);   // 2 indexes

    }

}

void Population::NextGen(){
    unsigned int i, j, k;
    // Формирование набора пар
    for( k = 0; k < descendants; k++) {
        i = pProb->sampling(Rndm());
        j = pProb->sampling(Rndm());

        /// скрещивание и мутация
        if (i != j)  popul[populIndexes[base + k]].Crossover(popul[populIndexes[i]], popul[populIndexes[j]]);
        else         popul[populIndexes[base + k]] = popul[populIndexes[j]];

        popul[populIndexes[base + k]].Mutation(p_mut);
        popul[populIndexes[base + k]].constructSolution();
    }
    ///   Вставка новых хромосом в массив
    for( k = 0; k < descendants; k++) ChromInsert(base - 1 + k, base + k);  ////***********
}

/***********************************************************************************************************/
/***********************************************************************************************************/
/// creating of  arrays and array of structures for solution construction
void createVehicleRoutes(){
    deliveryIndices = new int[nS];
    donePickups = new int[nS];

    ////***** creating vehicleRoute for storing solution***//
    vehicleRoutes = new vehicleRoute[nV];
    for(int i = 0; i < nV; ++i){
        vehicleRoutes[i].jobTypes = new string[nicks[i] * 2];
        vehicleRoutes[i].jobIds = new string[nicks[i] * 2];
        vehicleRoutes[i].arrTime = new double[nicks[i] * 2];
        vehicleRoutes[i].endTime = new double[nicks[i] * 2];

    }
}

/// initializing of arrays and array of structures for solution construction
void initVehicleRoutes(){
    for( int i = 0; i < nS; ++i) deliveryIndices[i] = -1;
    for(int i = 0; i < nS; ++i) donePickups[i] = -1;

    ////***** initializing VehicleRoutes***//
    /// filling initial value of currentCapacity, cost ets. for vehicles
    for(int i = 0; i < nV; i++){
        vehicleRoutes[i].currentCapacity = Vehicles[i].capacity;
        vehicleRoutes[i].cost = 0;
        vehicleRoutes[i].prevLocation = -1;       /// -1 means vehicle is at base
        vehicleRoutes[i].currNumPickups = 0;      /// current number of pickups
        vehicleRoutes[i].routePoint = -1;
        vehicleRoutes[i].startAct = vehicleRoutes[i].endAct = 0;
    }

    for(int i = 0; i < nV; ++i){
        for(int j = 0; j < nicks[i] * 2; ++j){
            vehicleRoutes[i].jobTypes[j] = "-1";
            vehicleRoutes[i].jobIds[j] = "-1";
            vehicleRoutes[i].arrTime[j] = 0;
            vehicleRoutes[i].endTime[j] = 0;
        }
    }
}

/// deleting the arrays for solution construction
void delVehicleRoutes(){

    delete[] deliveryIndices;
    delete[] donePickups;
    ////***** deleting vehicleRoutes ***//
    for(int i = 0; i < nV; ++i){
        delete[] vehicleRoutes[i].arrTime;
        delete[] vehicleRoutes[i].endTime;
        delete[] vehicleRoutes[i].jobTypes;
        delete[] vehicleRoutes[i].jobIds;
    }
    delete[] vehicleRoutes;
}

/// Writing solution into the json file
void writeVehicleRoutes(double C, char* filename){
    ////***** writing ***//
    Json::Value res;

    res["cost"] = C;

    for(int i = 0; i < nV; ++i){
        res["routes"][i]["vehicleId"] = Vehicles[i].id;
        res["routes"][i]["start"] = vehicleRoutes[i].startAct;
        res["routes"][i]["end"] = vehicleRoutes[i].endAct;
        for(int j = 0; j < nicks[i] * 2; ++j){
            res["routes"][i]["act"][j]["type"] = vehicleRoutes[i].jobTypes[j];
            res["routes"][i]["act"][j]["jobId"] = vehicleRoutes[i].jobIds[j];
            res["routes"][i]["act"][j]["arrTime"] = vehicleRoutes[i].arrTime[j];
            res["routes"][i]["act"][j]["endTime"] = vehicleRoutes[i].endTime[j];
        }
    }

    ofstream output_file(filename, ofstream::out);

    output_file << res;
    output_file.close();
}

