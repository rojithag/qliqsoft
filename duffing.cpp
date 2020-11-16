
///////////////////////////////////////
///      Rojitha Goonesekere        ///
/////Duffing OSC Simulator with FFT////
///////////////////////////////////////

#include<iostream>
#include<iomanip>
#include<string>
#include<fstream>

//FFT headers//
#include <complex>
#define MAX 200

//Initialize value for Pi
const double M_PI = 3.141592653589793;


using namespace std;



//////////////////////////
//Begin main block here//
/////////////////////////

int main() {
	
	//open a stream to write to an output file//
	ofstream outfile;
	ofstream outfile2;
	ofstream outfile3;
	outfile.open("ProgramData.txt");
	outfile2.open("NodePoints.txt");
	outfile3.open("FFT.txt");

	//Declaring all the constants//
	
	double k = 10000;
	double f = 810.0644;

	double w = (2 * M_PI*f / k);

	const double Te = 2 * M_PI / w;

	//Declaring Tp period//
	double tp;

	//double a = -10.0, b = 10.0, d = 0.5, v = 10, T = 1000 * Te;

	double a = -1, b = 1, d = 0.25, v = 0.365, T = 1000 * Te;

	const double delT = Te / 1024;

	//Declaring all the variables to a starting value//
	double e = 2, tol = 1.0e-10;

	double x = -1.0622, y = 0.0617, dx = 0, dy = 0;

	double xi = 0, yi = 0;

	//Declare period found boolean//
	bool PeriodFound1 = false;
	bool nochange = true;

	//Initializing the 100x3 matrix to store node values//
	double storage[100][3] = { 0 };

	//Variable for standard deviation//
	double stdd;

	//Declaring FFT variables here//
	int arrSize = 100;
	int sampleRate = 1;
	complex<double> vec[MAX];

	int row = 0;

	//Start all the calulations//
	for (double t = M_PI/w; t <= T; t += delT) 
	{

		e = 2.0;

		while (e >= tol) 
		{

			dx = x + delT*((y + yi) / 2);
			dy = y + delT*(-a*((x + xi) / 2) - b*pow(((x + xi) / 2), 3) -
				d*((y + yi) / 2) + v*cos(w*(t + delT / 2)));

			e = pow((dx - xi), 2) + pow((dy - yi), 2);

			xi = dx; yi = dy;

		}

		x = xi; y = yi;

		//Calculates the Mod of t/Te
		double mod = fmod(t, Te);

		//Puts the first set in the buffer//
		if (abs(mod) <= 1.0e-10 || abs(mod - Te) <= 1.0e-10)
		{

			//if a period is found output node points on a different file
			if (PeriodFound1 == true)
			{
				outfile2.precision(12);
				//if period found print the node points on the output file//
				outfile2 << t << ' ' << x
					<< ' ' << y << endl;
			}


			if (row != 0)
			{
				if (PeriodFound1 == false) 
				{
					//Find periodic//
					for (int rowChk = (row - 1); rowChk >= (((row - 100) < 0) ? 0 : (row - 100)); rowChk--)

					{

						//calculating standard deviation here//
						if (rowChk != 0)
						{
							double prevX = storage[rowChk % 100][1];
							double prevY = storage[rowChk % 100][2];

							stdd = sqrt((pow((x - prevX), 2) + pow((y - prevY), 2)) / 2);

							if (abs(stdd) < 1.0e-4)
							{
								PeriodFound1 = true;
								tp = t - storage[rowChk % 100][0];
							}

						}

					}
				}
			}

			//adding all the values to a circular array
			storage[row % 100][0] = t;
			storage[row % 100][1] = x;
			storage[row % 100][2] = y;


			//increament row//
			row++;
		}

		if (PeriodFound1 & nochange) 
		{
			t = 0.0, T = 2 * tp;
			nochange = false;
		}

		if (PeriodFound1 == true) 
		{
			outfile.precision(12);
			//if period found print the values on the output file//
			outfile << t << ' ' << x
				<< ' ' << y << endl;

			//Outputing the FFT values//
			for (int j = 0; j < arrSize; j++)
				outfile3 << vec[j] << endl;

		}
	}
}



//////////////////////////////////////
////Doing all the work for FTT here//
/////////////////////////////////////
//
//
//int log2(int N)    /*function to calculate the log2(.) of int numbers*/
//{
//	int k = N, i = 0;
//	while (k) {
//		k >>= 1;
//		i++;
//	}
//	return i - 1;
//}
//
//int check(int n)    //checking if the number of element is a power of 2
//{
//	return n > 0 && (n & (n - 1)) == 0;
//}
//
//int reverse(int N, int n)    //calculating revers number
//{
//	int j, p = 0;
//	for (j = 1; j <= log2(N); j++) {
//		if (n & (1 << (log2(N) - j)))
//			p |= 1 << (j - 1);
//	}
//	return p;
//}
//
//void ordina(complex<double>* f1, int N) //using the reverse order in the array
//{
//	complex<double> f2[MAX];
//	for (int i = 0; i < N; i++)
//		f2[i] = f1[reverse(N, i)];
//	for (int j = 0; j < N; j++)
//		f1[j] = f2[j];
//}
//
//void transform(complex<double>* f, int N) //
//{
//	ordina(f, N);    //first: reverse order
//	complex<double> *W;
//	W = (complex<double> *)malloc(N / 2 * sizeof(complex<double>));
//	W[1] = polar(1., -2. * M_PI / N);
//	W[0] = 1;
//	for (int i = 2; i < N / 2; i++)
//		W[i] = pow(W[1], i);
//	int n = 1;
//	int a = N / 2;
//	for (int j = 0; j < log2(N); j++) {
//		for (int i = 0; i < N; i++) {
//			if (!(i & n)) {
//				complex<double> temp = f[i];
//				complex<double> Temp = W[(i * a) % (n * a)] * f[i + n];
//				f[i] = temp + Temp;
//				f[i + n] = temp - Temp;
//			}
//		}
//		n *= 2;
//		a = a / 2;
//	}
//}
//
//void FFT(complex<double>* f, int N, double d)
//{
//	transform(f, N);
//	for (int i = 0; i < N; i++)
//		f[i] *= d; //multiplying by step
//}

////cooking up the FFT//
//
//for (int i = 0; i < arrSize; i++) {
//	vec[i] = storage[row % 100][1];
//}
//
//FFT(vec, arrSize, sampleRate);
