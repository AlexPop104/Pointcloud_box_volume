#include <iostream>
#include <math.h>
#include <fstream>
#include <string>  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>


int 
main (int argc, char** argv){
	
int i,j,aux1,aux2;

 std::cout<<"i=";
 std::cin>>i;
 std::cout<<"j=";
 std::cin>>j;
 
 aux1=i;
 aux2=j;
 
 std::stringstream fisier_i;
 std::stringstream fisier_j;
 
 float Coeficienti[2][5];
  
  fisier_i <<"Coeficienti_Linie"<<i<<".txt";
  
  fisier_j <<"Coeficienti_Linie"<<j<<".txt";
  
  
  std::ifstream myfile_i;
  myfile_i.open(fisier_i.str());
	
int t=0;	


	
for (std::string line; std::getline(myfile_i,line);){
	 std::istringstream in_i(line);
	 in_i>>Coeficienti[0][t];
	 std::cout<<Coeficienti[0][t]<<" ";
	 t++;
	}
	

std::cout<<"\n";


	std::ifstream myfile_j;
	myfile_j.open(fisier_j.str());

t=0;	

for (std::string line; std::getline(myfile_j,line);){
	 std::istringstream in_j(line);
	 in_j>>Coeficienti[1][t];
	 std::cout<<Coeficienti[1][t]<<" ";
	 t++;
	}
	
myfile_j.close();
	
std::cout<<"\n";

float a[2];
float b[2];
float c[2];
float x[2];
float y[2];

for(i=0;i<2;i++){
	a[i]=Coeficienti[i][0];
	b[i]=Coeficienti[i][1];
	c[i]=Coeficienti[i][2];
	y[i]=Coeficienti[i][3];
	x[i]=Coeficienti[i][4];
}



bool ok=1;

float epsilon=1/100000;

for (i=0;i<2;i++){
	for(j=0;j<3;j++){
		if ( (Coeficienti[i][j]==0+epsilon) || (Coeficienti[i][j]==0-epsilon) ) {
			ok=0;
			}
		}
	}


float raport_a= a[0]/a[1];
float raport_b= b[0]/b[1];
float raport_c= c[0]/c[1];

for (i=0;i<2;i++){
	std::cout<<"a_"<<i<<"="<<a[i]<<" "<<"b_"<<i<<"="<<b[i]<<" "<<"c_"<<i<<"="<<c[i]<<"y_"<<i<<"="<<y[i]<<"x_"<<i<<"="<<x[i]<<"\n";
}




float x_m;
float y_m;
float z_m;



if (ok==0)
	std::cout<<"Linia nu este bine definita";
else
	{
		if( (raport_a==raport_b) &&(raport_a==raport_c) && (raport_b==raport_c)){
			std::cout<<"Planurile sunt paralele";
			}
			else 
			{
				x_m= (c[1]*a[0]*x[1]-c[0]*a[1]*x[0])/(c[1]*a[0]+c[0]*a[1]);
				y_m= (c[1]*b[0]*y[1]-c[0]*b[1]*y[0])/(c[1]*b[0]+c[0]*b[1]);
				z_m= c[0]/a[0] *(x_m-x[0]);
				
				
				std::cout<<"x_m="<<x_m<<"\n";	
				std::cout<<"y_m="<<y_m<<"\n";
				std::cout<<"z_m="<<z_m<<"\n";

				
				std::stringstream fisier;
  
				fisier <<"Coordonate_punct_"<<aux1<<"_"<<aux2<<".txt";
  
  
				std::ofstream myfile;
				myfile.open (fisier.str());
  
				myfile<<x_m<<"\n"	
						<<y_m<<"\n"
						<<z_m<<"\n";
						myfile.close();
				
				
				}
	}




 return (0);
}
