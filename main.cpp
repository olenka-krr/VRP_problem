
#include <cmath>

#include "json/json.h"
#include "json/json-forwards.h"

#include "VOGtest.h"
#include "EVFalg.h"

using namespace std;

bool   EVF_alg(Chromosome& p_out_Chromosome,
               unsigned int Achrom, unsigned int Apair, unsigned int Agen,
               double pMut,  double ProbH, double ProbS);

vehicle* Vehicles;    /// array of vehicles
int nV;               /// number of vehicles
shipment* Shipments;  /// array of shipments
int nS;               /// number of shipments

unsigned int Chromosome::N;  /// chromosome`s length = 2 * nS

double** DistanceMatrix;            /// distances between client`s points, ((2*nS) * (2*nS))
double** VehicleDistanceMatrix;     /// distances between vehicles and client`s points, (nV * (2 * nS))

int* deliveryIndices;       /// array of nS-size for finding the delivery in permutation, which corresponds given pickup

unsigned int* nicks;        /// marks dividing shipments for vehicles, array of (nV - 1)

int* donePickups;           /// array of vehicle`s Id, (nS)

void readInputData(char*);          /// Filling arrays of vehicles and shipments
void fillDistanceMatrices();   /// Distance_between_client_points matrix is (2*nS) * (2*nS), even numbers - pickups, odd - arrivals.

void createVehicleRoutes();        /// creating of arrays for vehicle routes
void delVehicleRoutes();          /// delete array for vehicle routes
void writeVehicleRoutes(double, char*);  /// write to json vehicle routes


int main(int N, char* f[])
{
    char * fin, * fout;

    if( N < 2) fin = "vrp_task.json";
    else       fin = f[1];

    if( N < 3) fout = "vrp_task_solution.json";
    else       fout = f[2];


    /// Reading input data
    /// filling Vehicles[nV], Shipments[nS]
    readInputData(fin);

    /// Distance matrix is (2*nS) * (2*nS), VehicleDistanceMatrix is (nV * (2*nS))
    /// even numbers - pickups, odd - deliveries.
    fillDistanceMatrices();


    nicks = new unsigned int[nV];  /// Number of shipments per vehicle

    if ( N < 4){
        nicks[0] = 35;
        nicks[1] = 25;
        nicks[2] = 25;
        nicks[3] = 15;
        nicks[4] = 0;
    } else {
         Json::Value obj;
         ifstream nicksFile(f[3], ifstream::binary);
         nicksFile >> obj;
         nicksFile.close();

         Json::Value nc = obj["nicks"];
         int sum = 0;
         for(int i = 0; i < nc.size(); i++) {
            nicks[i] = nc[i].asInt();   //  obj["nicks"][i].asInt();
            sum += nicks[i];
         }
         if((sum != nS) || nc.size() != nV) {cout << "Error in " << f[3] << endl; return 1;}
    }

    cout << "\nnicks:\n";
    for(int i = 0; i < nV; ++i) cout << nicks[i] << ' ';
    cout << endl << endl;


    /// ************ Evolutionary algorithm
    //////////////**********************************
    // 600 + 100 * 500 = 50 600;  100 000 + 50 000 * 2 500 = 125 100 000;  1000 + 200 * 565 = 114 000; 50 000 + 20 000 * 1 250 = 250 050 000;

    unsigned int Achrom = 1000;    /// Размер популяции               // 100 000; 50 000; 1000; 100 000;
    unsigned int Apair = 500;      /// Количество пар для кроссовера  // 50 000; 20 000; 200;
    unsigned int Agen  = 50;       /// Количество поколений           // 625; 394; 500; 1250; 565; 2 500;

    if ( N >= 5){
        Json::Value obj;
        ifstream ParamFile(f[4], ifstream::binary);
        ParamFile >> obj;
        ParamFile.close();

        Achrom = obj["Achrom"].asInt();
        Apair = obj["Apair"].asInt();
        Agen = obj["Agen"].asInt();
    }

    double pMut  = 0.1;     /// Вероятность мутации
    double ProbH = 1;       /// Высота в треугольном распределении (доля от Achrom)
    double ProbS = 0.1;     /// Смещение в треугольном распределении (доля от Achrom)
    cout << "Achrom = " << Achrom << " Apair = " << Apair << " Agen = " << Agen << "\n\n";

    //cin.get();

    srand((unsigned int) time(NULL)/2);  /// Запуск генератора случайных чисел

    Chromosome::N = 2 * nS;  /// pickups + deliveries

    //cout << "Chromosome::N = " << Chromosome::N << "\n\n";

    Chromosome popBest;
    popBest.UniformDistrRandPermutN();
    createVehicleRoutes();
    if(!popBest.constructSolution()) /// cost of the first chromosome
        exit(0);

    double res = popBest.cost;
    cout << "start result = " << popBest.cost << "\n\n";

    writeVehicleRoutes(popBest.cost, fout);

    EVF_alg(popBest, Achrom, Apair, Agen, pMut, ProbH, ProbS);

    cout << "\n**************Solution*****************\n";
    cout << "EVF_result = " << popBest.cost << "\n\n";
    if( popBest.cost < res ){
        res = popBest.cost;
        writeVehicleRoutes(popBest.cost, fout);
        cout << "result = " << res << "\n\n";
    }

    delete[] Shipments;
    delete[] Vehicles;

    for(int i = 0; i < nS; ++i) delete[] DistanceMatrix[i];
    delete[] DistanceMatrix;

    for(int i = 0; i < nV; ++i) delete[] VehicleDistanceMatrix[i];
    delete[] VehicleDistanceMatrix;

    delVehicleRoutes();


    cout << "\n------------Ok-----------\n";
    return 0;
}

/// Reading input data
void readInputData(char* filename){
    Json::Value root;
    ifstream input_file(filename, ifstream::binary);

    input_file >> root;
    input_file.close();

    const Json::Value vhs = root["vehicles"];

    nV = vhs.size();
    cout << "\nnumber of vehicles = " << nV << endl;

    Vehicles = new vehicle[nV];

    for(int i = 0; i < nV; i++){
        Vehicles[i].id = root["vehicles"][i]["id"].asString();
        Vehicles[i].capacity = root["vehicles"][i]["capacity"].asInt();
        Vehicles[i].distance_price = root["vehicles"][i]["distance_price"].asInt();
        Vehicles[i].baseX = root["vehicles"][i]["coord"]["x"].asDouble();
        Vehicles[i].baseY = root["vehicles"][i]["coord"]["y"].asDouble();
    }

    const Json::Value shs = root["shipments"];
    nS = shs.size();
    cout << "number of shipments = " << nS << endl << endl;

    Shipments = new shipment[nS];

    for(int i = 0; i < nS; i++){
        Shipments[i].id = root["shipments"][i]["id"].asString();
        Shipments[i].pickupX = root["shipments"][i]["pickup"]["x"].asDouble();
        Shipments[i].pickupY = root["shipments"][i]["pickup"]["y"].asDouble();
        Shipments[i].deliveryX = root["shipments"][i]["delivery"]["x"].asDouble();
        Shipments[i].deliveryY = root["shipments"][i]["delivery"]["y"].asDouble();
        Shipments[i].capacityDemand = root["shipments"][i]["capacityDemand"].asInt();
    }
}

/// filling the distance`s matrices
void fillDistanceMatrices(){
    /// Distance_between_client_points matrix is ((2*nS) * (2*nS))
    /// even numbers - pickups, odd - deliveries.
    DistanceMatrix = new double*[2 * nS];
    for(int i = 0; i < 2 * nS; ++i) DistanceMatrix[i] = new double[2 * nS];

    for(int i = 0; i < nS; ++i){
        DistanceMatrix[2 * i][2 * i] = 0;
        DistanceMatrix[2 * i + 1][2 * i + 1] = 0;
        DistanceMatrix[2 * i][2 * i + 1] = DistanceMatrix[2 * i + 1][2 * i] = sqrt(pow(Shipments[i].pickupX - Shipments[i].deliveryX, 2.) + pow(Shipments[i].pickupY - Shipments[i].deliveryY, 2.));
        for (int j = i + 1; j < nS; ++j){
            DistanceMatrix[2 * i][2 * j] = DistanceMatrix[2 * j][2 * i] = sqrt(pow(Shipments[i].pickupX - Shipments[j].pickupX, 2.) + pow(Shipments[i].pickupY - Shipments[j].pickupY, 2.));
            DistanceMatrix[2 * i][2 * j + 1] = DistanceMatrix[2 * j + 1][2 * i] = sqrt(pow(Shipments[i].pickupX - Shipments[j].deliveryX, 2.) + pow(Shipments[i].pickupY - Shipments[j].deliveryY, 2.));
            DistanceMatrix[2 * i + 1][2 * j] = DistanceMatrix[2 * j][2 * i + 1] = sqrt(pow(Shipments[i].deliveryX - Shipments[j].pickupX, 2.) + pow(Shipments[i].deliveryY - Shipments[j].pickupY, 2.));
            DistanceMatrix[2 * i + 1][2 * j + 1] = DistanceMatrix[2 * j + 1][2 * i + 1] = sqrt(pow(Shipments[i].deliveryX - Shipments[j].deliveryX, 2.) + pow(Shipments[i].deliveryY - Shipments[j].deliveryY, 2.));
        }
    }

    /// Distances_between_vehicles_and_client`s_points matrix is (nV * (2*nS))
    /// even numbers - pickups, odd - deliveries.
    VehicleDistanceMatrix = new double*[nV];
    for(int i = 0; i < nV; ++i) VehicleDistanceMatrix[i] = new double[2 * nS];

    for(int i = 0; i < nV; ++i)
        for (int j = 0; j < 2 * nS; ++j){
            VehicleDistanceMatrix[i][2 * j] = sqrt(pow(Vehicles[i].baseX - Shipments[j].pickupX, 2.) + pow(Vehicles[i].baseY - Shipments[j].pickupY, 2.));
            VehicleDistanceMatrix[i][2 * j + 1] = sqrt(pow(Vehicles[i].baseX - Shipments[j].deliveryX, 2.) + pow(Vehicles[i].baseY - Shipments[j].deliveryY, 2.));

        }
}


