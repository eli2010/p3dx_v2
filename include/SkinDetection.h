
#ifndef FACE_SKINDETECTION_H
#define FACE_SKINDETECTION_H

#pragma once

#include "irpImage/UniImageAll.h"

namespace face
{

class SkinDetection
{
public:
	/// Constructor.
	SkinDetection();
	/// Destructor.
	virtual ~SkinDetection();
    /// set the probility of ever pixels,that the is the skinpixels.
	void SetPSkin( double p);
    /// save and load the probability of LUT
	bool Save(const char* st, const char* st_no);
	bool Load(const char* st, const char* st_no);
	/// Generates the LUT,which contains the ratio of the number skin pixels that have a value [r,g,b] to the total number of skin pixels.
	bool	GenerateLUT(const std::vector<img::ByteRGBImage> & vecImg, const std::vector<img::ByteRGBImage> &vecMask );

	/// Apply skin detection
    img::ByteRGBImage 	Detect(const img::ByteRGBImage & imgInput);
	/// Get the probablility for every pixel.
 	img::DoubleImage 	GetProbability(const img::ByteRGBImage & imgInput);

private :

	double      m_P_RGB;
	double		m_P_skin;
	bool		m_LUTgenerated;
	double *    m_LUT_Problitiy_RGB_skin;
	double *    m_LUT_Problitiy_RGB_no_skin;

	
};
} // namespace

#endif 

  
  
  
  
  
  
  
  
  
  
  
 