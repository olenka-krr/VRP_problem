//#include <cstdlib>
#include "EVFalg.h"
using namespace std;


//double Rndm(); // returns uniformly distributed on [0; 1) random number

/************
�������� ���-���������
*************/

bool   EVF_alg(Chromosome& out_Chromosome,
               unsigned int Achrom, unsigned int Apair, unsigned int Agen,
               double pMut,  double ProbH, double ProbS) {  //, double t_lim

    /*****************
    out_Chromosome - ���������-����� (������-������������). ����������� - ����� ���������

    ��������� ������������� ���������
    Achrom - ������ ���������
    Apair - ���������� ��� ��� �����������
    Agen  - ���������� ���������
    pMut - ����������� �������
    Pr�bH, Pr�bS - ��������� ������������� ������������ ������ ���
    Pr�bH - ������������� ������ ������, ProbS - ������������� ��������
    Achrom*ProbH - ���������� ������, Achrom*ProbS - ������������� ��������
    ***************/

    cout << "*** EVF: begins......\n";

    Population P(Achrom, Apair, ProbH, ProbS, pMut);   // ��������� (base + descendants)

    cout << "*** EVF: Beginning population\n";

    P.setElement0(out_Chromosome);

    P.InitPop();

    cout << "*** EVF: Generation growing\n";
    // ���� ���������
    for (unsigned int i = 1; i <= Agen; i++) {
        cout << '*';
        //cout << "\nGeneration " << i << " from " << Agen << endl;

        P.NextGen();
        //for (int j = 0; j < Achrom + Apair; j++) P.getElementByIndx(j, out_Chromosome);


    }
    cout << "\n";

    //cout << "\n*** EVF: ended. Population:\n";
    for (int j = 0; j < Achrom + Apair; j++){
        P.getElementByIndx(j, out_Chromosome);
        //cout << j << ": " << out_Chromosome.cost << endl;
    }
    P.getElementByIndx(0, out_Chromosome);



    cout << "\n*** EVF: ended......\n";
    return true;
}

/**************************************************************************************************************/

int Vmax(int x, int y)  { return (x > y) ? x : y; }

double Rndm(){
    return (double) (rand() / (double)(RAND_MAX + 1));
}
