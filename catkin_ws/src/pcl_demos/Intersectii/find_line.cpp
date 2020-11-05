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
 
 float Coeficienti[2][4];
  
  fisier_i <<"coeficienti_plan_"<<i<<".txt";
  
  fisier_j <<"coeficienti_plan_"<<j<<".txt";
  
  
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
float d[2];

for(i=0;i<2;i++){
	a[i]=Coeficienti[i][0];
	b[i]=Coeficienti[i][1];
	c[i]=Coeficienti[i][2];
	d[i]=Coeficienti[i][3];
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
	std::cout<<"a_"<<i<<"="<<a[i]<<" "<<"b_"<<i<<"="<<b[i]<<" "<<"c_"<<i<<"="<<c[i]<<"d_"<<i<<"="<<d[i]<<"\n";
}




float x0;
float y0;
float a_s;
float b_s;
float c_s;



if (ok==0)
	std::cout<<"Planul nu este bine definit";
else
	{
		if( (raport_a==raport_b) &&(raport_a==raport_c) && (raport_b==raport_c)){
			std::cout<<"Planurile sunt paralele";
			}
			else 
			{
				a_s= (b[0]*c[1]-b[1]*c[0]);
				b_s= (a[1]*c[0]-a[0]*c[1]);
				c_s= (a[0]*b[1]-a[1]*b[0]);
				
				y0= -(a[1]*d[0]-a[0]*d[1])/ (a[1]*b[0]-a[0]*b[1]);
				x0= (d[0] +b[0]*y0)/ (-a[0]);
				
				
				std::cout<<"a_s="<<a_s<<"\n";	
				std::cout<<"b_s="<<b_s<<"\n";
				std::cout<<"c_s="<<c_s<<"\n";

				std::cout<<"y0="<<y0<<"\n";
				std::cout<<"x0="<<x0<<"\n";
				
				std::stringstream fisier;
  
				fisier <<"Coeficienti_Linie"<<aux1<<"_"<<aux2<<".txt";
  
  
				std::ofstream myfile;
				myfile.open (fisier.str());
  
				myfile<<a_s<<"\n"
						<<b_s<<"\n"
						<<c_s<<"\n"
						<<y0<<"\n"
						<<x0<<"\n";
						myfile.close();
				
				
				}
	}




 return (0);
}
