#include <Continuity/eval_continuity.hpp>

double eval_continuity(KDL::Jacobian& J1,KDL::Jacobian& J2)
{
	// double mean_J1 = 0;
	// double mean_J2 = 0;
	// double a=0; double b=0; double c=0;
	// double corr;

	// for (int i=0;i<J1.rows();i++)
	// {
	// 	for (int j=0;j<J1.columns();j++)
	// 	{
	// 		mean_J1 += J1(i,j);
	// 		mean_J2 += J2(i,j);
	// 	}
	// }
	// mean_J1 /= (J1.rows()+J1.columns());

	// for (int i=0;i<J2.rows();i++)
	// {
	// 	for (int j=0;j<J2.columns();j++)
	// 	{
	// 		a += (J1(i,j)-mean_J1)*(J2(i,j)-mean_J2); // Subtract from mean and cross multiple
	// 		b += pow((J1(i,j)-mean_J1),2); // Subtract from mean and square
	// 		c += pow((J2(i,j)-mean_J2),2); // Subtract from mean and square
	// 	}
	// }
	// mean_J2 /= (J2.rows()+J2.columns());

	// corr = a / sqrt(b*c);
	
	// return corr;

	return 1.0;
};