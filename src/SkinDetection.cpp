#include "SkinDetection.h"

namespace face
{

#define NUMBINS 32
#define BINSCALE 8  //256/NUMBINS

SkinDetection::SkinDetection()
{  
	m_P_RGB= 0.4 ;
	m_P_skin = 0.1;
	m_LUT_Problitiy_RGB_skin=NULL;
	m_LUT_Problitiy_RGB_no_skin=NULL;

	m_LUTgenerated = false;
}

SkinDetection::~SkinDetection()
{
	if (m_LUT_Problitiy_RGB_skin != NULL) delete [] m_LUT_Problitiy_RGB_skin;
	if (m_LUT_Problitiy_RGB_no_skin != NULL) delete [] m_LUT_Problitiy_RGB_no_skin;
}
bool SkinDetection::Save(const char* st, const char* st_no)
/// Only the units in the LUT,which have no Null probility,could be saved.
/// The position of these units could be also saved.
{
	std::fstream file, file_no;
	file.open(st, std::ios_base::out, std::ios_base::binary);
	file_no.open(st_no, std::ios_base::out, std::ios_base::binary);

	for (int a = 0; a < NUMBINS; a++)
		for (int b = 0; b < NUMBINS; b++)
			for (int c = 0; c <NUMBINS; c++)
			{
				if (m_LUT_Problitiy_RGB_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c] != 0.0)
				{
					file << a << " " << b << " " << c << " " << m_LUT_Problitiy_RGB_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c]  << std::endl;
				}
				if (m_LUT_Problitiy_RGB_no_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c] != 0.0)
				{
					file_no << a << " " << b << " " << c << " " <<m_LUT_Problitiy_RGB_no_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c]  << std::endl;
				}
			}

	file.close();
	file_no.close();
	return true;
}

bool SkinDetection::Load(const char* st, const char* st_no)
{   
	if (m_LUTgenerated != true)
	{
		m_LUT_Problitiy_RGB_skin	=new double[NUMBINS*NUMBINS*NUMBINS];
		m_LUT_Problitiy_RGB_no_skin	=new double[NUMBINS*NUMBINS*NUMBINS];
		m_LUTgenerated = true;
	}

	std::fstream file, file_no;
	file.open(st, std::ios_base::in, std::ios_base::binary);
	file_no.open(st_no, std::ios_base::in, std::ios_base::binary);

	for (int a = 0; a < NUMBINS; a++)
		for (int b = 0; b < NUMBINS; b++)
			for (int c = 0; c < NUMBINS; c++)
			{
				m_LUT_Problitiy_RGB_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c] = 0.0 ;
				m_LUT_Problitiy_RGB_no_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c] = 0.0  ;
			}

	while (!file.eof())
	{
		int a, b, c ;
		file >> a >> b  >> c;
		//std::cout << a << " " << b << " " << c << std::endl;
		file >>m_LUT_Problitiy_RGB_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c];
	}
	while (!file_no.eof())
	{
		int a, b, c ;
		file_no >> a >> b  >> c;
		file_no >>m_LUT_Problitiy_RGB_no_skin[a*(NUMBINS*NUMBINS)+b*NUMBINS+c];
	}
   	file.close();
	file_no.close();
	return true;
}

bool SkinDetection::GenerateLUT(const std::vector<img::ByteRGBImage> & vecImg, const std::vector<img::ByteRGBImage> &vecMask )
{	
	int number = 0;
	int number_no = 0;
//Define two LUTs.
	if (m_LUTgenerated != true)
	{
		m_LUT_Problitiy_RGB_skin	=new double[NUMBINS*NUMBINS*NUMBINS];
		m_LUT_Problitiy_RGB_no_skin	=new double[NUMBINS*NUMBINS*NUMBINS];
		m_LUTgenerated = true;
	}
///Initiate these two LUTs.
	for(int a=0;a<NUMBINS;a++)
		for(int b=0;b<NUMBINS;b++)
			for(int c=0;c<NUMBINS;c++)
			{
				m_LUT_Problitiy_RGB_skin[a*(NUMBINS*NUMBINS) + b*NUMBINS +c]=0;
				m_LUT_Problitiy_RGB_no_skin[a*(NUMBINS*NUMBINS) + b*NUMBINS +c]=0;
			}
///Calculate the probilities in these two LUTs.
	int num = vecMask.size();
	for (int n = 0; n <num;n++)
	{
		for(int i=0 ; i<vecImg[n].GetSizeX(); i++)
		   for(int j=0 ; j<vecImg[n].GetSizeY(); j++)
		   {
			   if (vecMask[n].Pixel(i,j) != img::ByteRGB(255) )
			   {
					m_LUT_Problitiy_RGB_skin[(vecImg[n].Pixel(i,j).red/BINSCALE)*(NUMBINS*NUMBINS) 
						+ (vecImg[n].Pixel(i,j).green/BINSCALE)*NUMBINS 
						+ (vecImg[n].Pixel(i,j).blue/BINSCALE)]++;
					number++;
				} 
			   else
			   {
					m_LUT_Problitiy_RGB_no_skin[(vecImg[n].Pixel(i,j).red/BINSCALE)*(NUMBINS*NUMBINS) 
						+ (vecImg[n].Pixel(i,j).green/BINSCALE)*NUMBINS 
						+ (vecImg[n].Pixel(i,j).blue/BINSCALE)]++;
					number_no++;
			   }
		   }
	}

   for(int a=0;a<NUMBINS;a++)
		for(int b=0;b<NUMBINS;b++)
			for(int c=0;c<NUMBINS;c++)
			{
				int index = a*(NUMBINS*NUMBINS) + b*NUMBINS +c;
				m_LUT_Problitiy_RGB_skin[index]=(double)m_LUT_Problitiy_RGB_skin[index]/number;
				m_LUT_Problitiy_RGB_no_skin[index]=(double)m_LUT_Problitiy_RGB_no_skin[index]/number_no;
			}
	return true;
}

img::ByteRGBImage 	SkinDetection::Detect(const img::ByteRGBImage & imgInput)
/// Distinguish skinpixels.
/// set the background to black.
{

	img::ByteRGBImage Buffer = imgInput;
	for(int i=0 ; i<imgInput.GetSizeX(); i++)
		for(int j=0 ; j<imgInput.GetSizeY(); j++)
		{
			int index = (imgInput.Pixel(i,j).red/BINSCALE)*(NUMBINS*NUMBINS) 
				+ (imgInput.Pixel(i,j).green/BINSCALE)*NUMBINS 
				+(imgInput.Pixel(i,j).blue/BINSCALE);
			if ((m_LUT_Problitiy_RGB_skin[index] == 0 && m_LUT_Problitiy_RGB_no_skin[index] == 0))
				Buffer.Pixel(i,j) = img::ByteRGB(0,0,0);
			else if (m_LUT_Problitiy_RGB_skin[index]*m_P_skin<m_LUT_Problitiy_RGB_no_skin[index]*(1-m_P_skin))
				Buffer.Pixel(i,j) = img::ByteRGB(0,0,0);
	   }
	return Buffer;
}
img::DoubleImage 	SkinDetection::GetProbability(const img::ByteRGBImage & imgInput)
{  

	int width = imgInput.GetSizeX();
	int height = imgInput.GetSizeY();

   img::DoubleImage probMap ;
   probMap.SetSize( width, height);
   // Berechung der Wahrscheinlichkeit für jeder Pixel 
   for(int i=0 ; i< width; i++)
		   for(int j=0 ; j<height; j++)
		   {  int index = (imgInput.Pixel(i,j).red/BINSCALE)*(NUMBINS*NUMBINS) 
		   + (imgInput.Pixel(i,j).green/BINSCALE)*NUMBINS 
		   + (imgInput.Pixel(i,j).blue/BINSCALE);
			  probMap.Pixel(i,j)= (m_LUT_Problitiy_RGB_skin[index]*m_P_skin)/m_P_RGB;
		   }
   return probMap; 
}

} // namespace